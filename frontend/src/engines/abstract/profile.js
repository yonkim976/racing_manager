export const ABSTRACT_BROADCAST_RUNTIME_PROFILE = Object.freeze({
  family: 'abstract',
  liveBroadcast: true,
  resultOnly: false,
  rendererLabel: 'STRATEGY MAP',
  allowedSpeeds: Object.freeze([1, 2, 5]),
  physicsDevControls: false,
  telemetryAuthority: 'logical-events',
  presentationAuthority: 'derived-map-pose',
});

export const ABSTRACT_RESULT_RUNTIME_PROFILE = Object.freeze({
  family: 'abstract',
  liveBroadcast: false,
  resultOnly: true,
  rendererLabel: 'RESULT',
  allowedSpeeds: Object.freeze([]),
  physicsDevControls: false,
  telemetryAuthority: 'logical-result',
  presentationAuthority: 'none',
});
