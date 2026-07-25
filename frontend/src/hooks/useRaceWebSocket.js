import { useState, useEffect, useRef, useCallback } from 'react';
import ReconnectingWebSocket from 'reconnecting-websocket';

const WS_URL = `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/ws/race`;
const DASHBOARD_UPDATE_INTERVAL_MS = 100;
const POSE_PACKET_HEADER_BYTES = 11;

function decodePosePacketHeader(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < POSE_PACKET_HEADER_BYTES) {
    return null;
  }
  const view = new DataView(buffer);
  if (
    view.getUint8(0) !== 0x46
    || view.getUint8(1) !== 0x31
    || view.getUint8(2) !== 0x50
    || view.getUint8(3) !== 0x31
  ) {
    return null;
  }
  return {
    type: 'pose_tick_binary',
    physics_frame: view.getUint32(4, true),
    speed_multiplier: view.getUint8(8),
    paused: Boolean(view.getUint8(9)),
    pose_buffer: buffer,
  };
}

/**
 * Custom hook for managing WebSocket connection to the race server.
 * Handles connection lifecycle, message parsing, and command sending.
 */
export function useRaceWebSocket(shouldConnect = false) {
  const [raceInfo, setRaceInfo] = useState(null);
  const [raceState, setRaceState] = useState(null);
  const [events, setEvents] = useState([]);
  const [raceEnd, setRaceEnd] = useState(null);
  const [connected, setConnected] = useState(false);
  const [connectionState, setConnectionState] = useState('disconnected');
  const [transportStats, setTransportStats] = useState({
    kilobytesPerSecond: 0,
    messagesPerSecond: 0,
  });
  const wsRef = useRef(null);
  const eventsRef = useRef([]);
  const poseTickRef = useRef(null);
  const dashboardTickRef = useRef(null);
  const dashboardUpdatedAtRef = useRef(0);
  const clientEventSequenceRef = useRef(0);
  const historyByDriverRef = useRef(new Map());
  const timingByDriverRef = useRef(new Map());
  const transportWindowRef = useRef({
    startedAtMs: 0,
    bytes: 0,
    messages: 0,
  });

  useEffect(() => {
    if (!shouldConnect) {
      return undefined;
    }

    setConnectionState('connecting');

    const ws = new ReconnectingWebSocket(WS_URL, [], {
      maxRetries: 10,
      reconnectInterval: 2000,
      maxReconnectInterval: 10000,
    });
    ws.binaryType = 'arraybuffer';

    wsRef.current = ws;

    const ensureEventId = (evt) => {
      if (Number(evt?.event_id) > 0) return evt;
      clientEventSequenceRef.current += 1;
      return {
        ...evt,
        event_id: `client-${clientEventSequenceRef.current}`,
      };
    };

    const appendEvents = (incomingEvents) => {
      const preparedEvents = (incomingEvents || []).map(ensureEventId);
      if (!preparedEvents.length) return;
      const newEvents = [...eventsRef.current, ...preparedEvents].slice(-50);
      eventsRef.current = newEvents;
      setEvents(newEvents);
    };

    const appendEvent = (evt) => appendEvents([evt]);

    const attachHistoriesToFreshState = (state) => {
      if (!state?.positions) return state;
      state.positions.forEach((driver) => {
        driver.lap_history = historyByDriverRef.current.get(
          Number(driver.driver_id),
        ) || driver.lap_history || [];
        Object.assign(
          driver,
          timingByDriverRef.current.get(Number(driver.driver_id)) || {},
        );
      });
      return state;
    };

    const storeHistories = (histories, fullSnapshot = false) => {
      const changedDriverIds = new Set();
      (histories || []).forEach((entry) => {
        const driverId = Number(entry.driver_id);
        const incoming = entry.lap_history || [];
        const startIndex = Math.max(0, Number(entry.start_index || 0));
        const previous = historyByDriverRef.current.get(driverId) || [];
        const nextHistory = fullSnapshot || startIndex === 0
          ? incoming
          : [...previous.slice(0, startIndex), ...incoming];
        historyByDriverRef.current.set(driverId, nextHistory);
        changedDriverIds.add(driverId);
      });
      return changedDriverIds;
    };

    const storeTiming = (positions) => {
      const changedDriverIds = new Set();
      (positions || []).forEach((position) => {
        const driverId = Number(position.driver_id);
        timingByDriverRef.current.set(driverId, {
          last_mini_sector_time: position.last_mini_sector_time || 0,
          last_mini_sector_delta_to_best:
            position.last_mini_sector_delta_to_best ?? null,
          mini_sector_splits: position.mini_sector_splits || [],
          mini_sector_statuses: position.mini_sector_statuses || [],
        });
        changedDriverIds.add(driverId);
      });
      return changedDriverIds;
    };

    ws.addEventListener('open', () => {
      setConnected(true);
      setConnectionState('connected');
    });

    ws.addEventListener('close', () => {
      setConnected(false);
      setConnectionState('disconnected');
    });

    ws.addEventListener('error', () => {
      setConnected(false);
    });

    ws.addEventListener('message', (event) => {
      try {
        const receivedAtMs = performance.now();
        const transportWindow = transportWindowRef.current;
        if (transportWindow.startedAtMs <= 0) {
          transportWindow.startedAtMs = receivedAtMs;
        }
        transportWindow.bytes += typeof event.data === 'string'
          ? event.data.length
          : Number(event.data?.size || 0);
        transportWindow.messages += 1;
        const transportElapsedMs = receivedAtMs - transportWindow.startedAtMs;
        if (transportElapsedMs >= 1000) {
          setTransportStats({
            kilobytesPerSecond: (
              transportWindow.bytes / 1024 / (transportElapsedMs / 1000)
            ),
            messagesPerSecond: (
              transportWindow.messages / (transportElapsedMs / 1000)
            ),
          });
          transportWindowRef.current = {
            startedAtMs: receivedAtMs,
            bytes: 0,
            messages: 0,
          };
        }
        if (event.data instanceof ArrayBuffer) {
          const posePacket = decodePosePacketHeader(event.data);
          if (posePacket) {
            poseTickRef.current = posePacket;
          }
          return;
        }

        const data = JSON.parse(event.data);

        switch (data.type) {
          case 'race_info':
            setRaceInfo(data);
            setRaceEnd(null);
            break;

          case 'pose_tick':
            // Only compact poses reach the 30 Hz ref. Dashboard and event data
            // are separate messages, so high-frequency playback no longer
            // parses and discards a complete race snapshot every frame.
            poseTickRef.current = data;
            break;

          case 'race_state':
            {
              const mergedState = attachHistoriesToFreshState(data);
              dashboardTickRef.current = mergedState;
              dashboardUpdatedAtRef.current = performance.now();
              setRaceState(mergedState);
            }
            break;

          case 'race_history':
            {
              const changedDriverIds = storeHistories(
                data.histories,
                Boolean(data.full_snapshot),
              );
              setRaceState((previous) => {
                if (!previous?.positions || !changedDriverIds.size) return previous;
                const mergedState = {
                  ...previous,
                  positions: previous.positions.map((driver) => (
                    changedDriverIds.has(Number(driver.driver_id))
                      ? {
                        ...driver,
                        lap_history: historyByDriverRef.current.get(
                          Number(driver.driver_id),
                        ) || [],
                      }
                      : driver
                  )),
                };
                dashboardTickRef.current = mergedState;
                return mergedState;
              });
            }
            break;

          case 'race_timing':
            {
              const changedDriverIds = storeTiming(data.positions);
              setRaceState((previous) => {
                if (!previous?.positions || !changedDriverIds.size) return previous;
                const mergedState = {
                  ...previous,
                  positions: previous.positions.map((driver) => (
                    changedDriverIds.has(Number(driver.driver_id))
                      ? {
                        ...driver,
                        ...timingByDriverRef.current.get(
                          Number(driver.driver_id),
                        ),
                      }
                      : driver
                  )),
                };
                dashboardTickRef.current = mergedState;
                return mergedState;
              });
            }
            break;

          case 'race_events':
            appendEvents(data.events);
            break;

          case 'tick':
            // Backward-compatible handling for older servers.
            poseTickRef.current = data;
            storeHistories(
              (data.positions || []).map((driver) => ({
                driver_id: driver.driver_id,
                lap_history: driver.lap_history || [],
              })),
              true,
            );
            storeTiming(data.positions);
            {
              const now = performance.now();
              const previous = dashboardTickRef.current;
              const urgentStateChange = !previous
                || previous.paused !== data.paused
                || previous.speed_multiplier !== data.speed_multiplier
                || previous.race_phase !== data.race_phase
                || previous.start_sequence_phase !== data.start_sequence_phase;
              if (
                urgentStateChange
                || now - dashboardUpdatedAtRef.current >= DASHBOARD_UPDATE_INTERVAL_MS
              ) {
                const mergedState = attachHistoriesToFreshState(data);
                dashboardTickRef.current = mergedState;
                dashboardUpdatedAtRef.current = now;
                setRaceState(mergedState);
              }
            }
            if (data.events?.length > 0) {
              appendEvents(data.events);
            }
            break;

          case 'race_end':
            setRaceEnd(data);
            break;

          case 'command_ack':
            appendEvent({
              type: 'command_ack',
              driver: '',
              message: data.message || `Command accepted: ${data.command || 'ok'}`,
              message_ko: data.message_ko || '명령을 처리했습니다',
            });
            break;

          case 'command_error':
            appendEvent({
              type: 'command_error',
              driver: '',
              message: data.message || 'Command failed',
              message_ko: data.message_ko || '명령 처리에 실패했습니다',
            });
            break;

          default:
            break;
        }
      } catch (err) {
        console.error('Failed to parse WebSocket message:', err);
      }
    });

    return () => {
      ws.close(1000, 'Race screen closed');
      wsRef.current = null;
      setConnected(false);
      setConnectionState('disconnected');
    };
  }, [shouldConnect]);

  const sendCommand = useCallback((command) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(command));
    }
  }, []);

  const resetState = useCallback(() => {
    setRaceInfo(null);
    setRaceState(null);
    setEvents([]);
    setRaceEnd(null);
    eventsRef.current = [];
    poseTickRef.current = null;
    dashboardTickRef.current = null;
    dashboardUpdatedAtRef.current = 0;
    clientEventSequenceRef.current = 0;
    historyByDriverRef.current.clear();
    timingByDriverRef.current.clear();
    transportWindowRef.current = {
      startedAtMs: 0,
      bytes: 0,
      messages: 0,
    };
    setTransportStats({
      kilobytesPerSecond: 0,
      messagesPerSecond: 0,
    });
  }, []);

  return {
    raceInfo,
    raceState,
    poseTickRef,
    events,
    raceEnd,
    connected,
    connectionState,
    transportStats,
    sendCommand,
    resetState,
  };
}
