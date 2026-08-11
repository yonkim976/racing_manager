import { FULL_RUNTIME_PROFILE } from './full/profile.js';
import {
  ABSTRACT_BROADCAST_RUNTIME_PROFILE,
  ABSTRACT_RESULT_RUNTIME_PROFILE,
} from './abstract/profile.js';

const PROFILES = Object.freeze({
  FULL: FULL_RUNTIME_PROFILE,
  ABSTRACT_BROADCAST: ABSTRACT_BROADCAST_RUNTIME_PROFILE,
  ABSTRACT: ABSTRACT_RESULT_RUNTIME_PROFILE,
  ABSTRACT_INSTANT: ABSTRACT_RESULT_RUNTIME_PROFILE,
});

export function runtimeProfileForMode(mode = 'FULL') {
  const profile = PROFILES[String(mode || 'FULL').toUpperCase()];
  if (!profile) throw new Error(`Unsupported simulation mode: ${mode}`);
  return profile;
}

export const RUNTIME_PROFILES = PROFILES;
