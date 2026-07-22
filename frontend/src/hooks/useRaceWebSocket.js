import { useState, useEffect, useRef, useCallback } from 'react';
import ReconnectingWebSocket from 'reconnecting-websocket';

const WS_URL = `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/ws/race`;
const DASHBOARD_UPDATE_INTERVAL_MS = 100;

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
  const wsRef = useRef(null);
  const eventsRef = useRef([]);
  const poseTickRef = useRef(null);
  const dashboardTickRef = useRef(null);
  const dashboardUpdatedAtRef = useRef(0);

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

    wsRef.current = ws;

    const appendEvent = (evt) => {
      const newEvents = [...eventsRef.current, evt].slice(-50);
      eventsRef.current = newEvents;
      setEvents(newEvents);
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
        const data = JSON.parse(event.data);

        switch (data.type) {
          case 'race_info':
            setRaceInfo(data);
            setRaceEnd(null);
            break;

          case 'tick':
            // Canvas pose consumption is deliberately independent from React.
            // Every 30 Hz packet reaches the stable ref, while the timing and
            // strategy dashboard renders at 10 Hz. This keeps JSON/React work
            // from stealing frames from Pixi's 60 FPS playback loop.
            poseTickRef.current = data;
            {
              const now = performance.now();
              const previous = dashboardTickRef.current;
              const urgentStateChange = !previous
                || previous.paused !== data.paused
                || previous.speed_multiplier !== data.speed_multiplier
                || previous.race_phase !== data.race_phase
                || previous.start_sequence_phase !== data.start_sequence_phase
                || data.events?.length > 0;
              if (
                urgentStateChange
                || now - dashboardUpdatedAtRef.current >= DASHBOARD_UPDATE_INTERVAL_MS
              ) {
                dashboardTickRef.current = data;
                dashboardUpdatedAtRef.current = now;
                setRaceState(data);
              }
            }
            if (data.events?.length > 0) {
              const newEvents = [...eventsRef.current, ...data.events].slice(-50);
              eventsRef.current = newEvents;
              setEvents(newEvents);
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
      ws.close();
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
  }, []);

  return {
    raceInfo,
    raceState,
    poseTickRef,
    events,
    raceEnd,
    connected,
    connectionState,
    sendCommand,
    resetState,
  };
}
