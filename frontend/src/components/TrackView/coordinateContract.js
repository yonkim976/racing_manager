/**
 * Shared renderer <-> local-metric coordinate contract from RaceInfo.
 *
 * Backend world poses are local metres. Track paths are render coordinates;
 * the frame origin and metres-per-render-unit are the only conversion.
 */
export function localMetricToRenderPoint(xM, yM, coordinateFrame) {
  const metersPerRenderUnit = Math.max(
    1e-9,
    Number(coordinateFrame?.metersPerRenderUnit || 1),
  );
  return [
    Number(coordinateFrame?.originXRender || 0) + Number(xM || 0) / metersPerRenderUnit,
    Number(coordinateFrame?.originYRender || 0) + Number(yM || 0) / metersPerRenderUnit,
  ];
}

export function renderToLocalMetricPoint(coord, coordinateFrame) {
  const metersPerRenderUnit = Math.max(
    1e-9,
    Number(coordinateFrame?.metersPerRenderUnit || 1),
  );
  return [
    (Number(coord?.[0] || 0) - Number(coordinateFrame?.originXRender || 0))
      * metersPerRenderUnit,
    (Number(coord?.[1] || 0) - Number(coordinateFrame?.originYRender || 0))
      * metersPerRenderUnit,
  ];
}
