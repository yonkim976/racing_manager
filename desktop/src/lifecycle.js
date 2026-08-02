const TRANSITIONS = {
  BOOTING: new Set(['BACKEND_STARTING', 'FAILED', 'APP_SHUTTING_DOWN']),
  BACKEND_STARTING: new Set(['BACKEND_READY', 'FAILED', 'APP_SHUTTING_DOWN']),
  BACKEND_READY: new Set(['WINDOW_READY', 'FAILED', 'APP_SHUTTING_DOWN']),
  WINDOW_READY: new Set(['IDLE', 'RACING', 'FAILED', 'APP_SHUTTING_DOWN']),
  RACING: new Set(['RESULTS', 'DISPOSING_RACE', 'APP_SHUTTING_DOWN', 'FAILED']),
  RESULTS: new Set(['DISPOSING_RACE', 'APP_SHUTTING_DOWN', 'FAILED']),
  DISPOSING_RACE: new Set(['IDLE', 'RACING', 'APP_SHUTTING_DOWN', 'FAILED']),
  IDLE: new Set(['RACING', 'DISPOSING_RACE', 'APP_SHUTTING_DOWN', 'FAILED']),
  APP_SHUTTING_DOWN: new Set(['FAILED']),
  FAILED: new Set(['APP_SHUTTING_DOWN']),
};

class LifecycleController {
  constructor(initialState = 'BOOTING') {
    if (!TRANSITIONS[initialState]) throw new Error(`Unknown lifecycle state: ${initialState}`);
    this.state = initialState;
  }

  transition(nextState) {
    if (!TRANSITIONS[nextState]) throw new Error(`Unknown lifecycle state: ${nextState}`);
    if (nextState === this.state) return this.state;
    if (!TRANSITIONS[this.state].has(nextState)) {
      throw new Error(`Illegal lifecycle transition ${this.state} -> ${nextState}`);
    }
    this.state = nextState;
    return this.state;
  }
}

module.exports = { LifecycleController, TRANSITIONS };
