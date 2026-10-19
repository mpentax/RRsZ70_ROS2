import { useEffect, useRef } from 'react';
import { record } from './helpers.ts';

export interface MapCanvasProps {
  mapData: Record<string, unknown> | null;
  busy: boolean;
  onReadMap: () => void;
}

export function MapCanvas({ mapData, busy, onReadMap }: MapCanvasProps) {
  const mapCanvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = mapCanvasRef.current;
    if (!canvas || !mapData) return;
    const info = record(mapData.info);
    const width = Number(info.width) || 0;
    const height = Number(info.height) || 0;
    const data = Array.isArray(mapData.data) ? mapData.data as number[] : [];
    if (width <= 0 || height <= 0 || data.length < width * height) return;
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const img = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const val = data[(height - 1 - y) * width + x];
        const idx = (y * width + x) * 4;
        if (val === 100) { img.data[idx] = 166; img.data[idx + 1] = 227; img.data[idx + 2] = 203; img.data[idx + 3] = 255; }
        else if (val === 0) { img.data[idx] = 30; img.data[idx + 1] = 58; img.data[idx + 2] = 66; img.data[idx + 3] = 255; }
        else { img.data[idx] = 11; img.data[idx + 1] = 18; img.data[idx + 2] = 21; img.data[idx + 3] = 255; }
      }
    }
    ctx.putImageData(img, 0, 0);
  }, [mapData]);

  return <section className="card map-cache-card" style={{marginTop: 0}}>
    <div className="section-heading">
      <div>
        <p className="eyebrow">VENDOR V1 OCCUPANCY GRID</p>
        <h2>Map</h2>
      </div>
      <button className="quiet" disabled={busy} onClick={onReadMap}>Read ROS map</button>
    </div>
    <p className="caption">Shared OccupancyGrid; receipt time is not source freshness or localisation.</p>
    {mapData && (
      <div className="map-panel" style={{marginTop: 12, paddingTop: 12}}>
        <figure className="map-figure">
          <div className="map-image-wrap">
            <canvas ref={mapCanvasRef} style={{ display: 'block', width: '100%', maxHeight: '340px', imageRendering: 'pixelated' }} />
          </div>
          <figcaption className="caption">
            Grid {String(record(mapData.info).width ?? '?')}×{String(record(mapData.info).height ?? '?')} cells @ {Number(record(mapData.info).resolution ?? 0.05).toFixed(2)} m/cell
          </figcaption>
        </figure>
      </div>
    )}
  </section>;
}
