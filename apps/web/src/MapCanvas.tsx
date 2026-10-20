import { useEffect, useRef, type MouseEvent } from 'react';
import { record } from './helpers.ts';

export interface MapCanvasProps {
  mapData: Record<string, unknown> | null;
  busy: boolean;
  onReadMap: () => void;
  selectedTarget?: { x: number; y: number } | null;
  onSelectTarget?: (x: number, y: number) => void;
}

function unwrapGrid(mapData: Record<string, unknown> | null): Record<string, unknown> | null {
  if (!mapData) return null;
  const nested = record(mapData.map);
  return Object.keys(nested).length > 0 ? nested : mapData;
}

export function MapCanvas({ mapData, busy, onReadMap, selectedTarget, onSelectTarget }: MapCanvasProps) {
  const mapCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const grid = unwrapGrid(mapData);
  const info = record(grid?.info);
  const originPos = record(record(info.origin).position);
  const width = Number(info.width) || 0;
  const height = Number(info.height) || 0;
  const resolution = Number(info.resolution) || 0.05;
  const originX = Number(originPos.x) || 0;
  const originY = Number(originPos.y) || 0;

  useEffect(() => {
    const canvas = mapCanvasRef.current;
    if (!canvas || !grid) return;
    const data = Array.isArray(grid.data) ? grid.data as number[] : [];
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
    if (selectedTarget && Number.isFinite(selectedTarget.x) && Number.isFinite(selectedTarget.y)) {
      const px = (selectedTarget.x - originX) / resolution;
      const py = height - 1 - (selectedTarget.y - originY) / resolution;
      if (px >= 0 && px <= width && py >= 0 && py <= height) {
        ctx.strokeStyle = '#ff6b6b';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, Math.PI * 2);
        ctx.moveTo(px - 5, py); ctx.lineTo(px + 5, py);
        ctx.moveTo(px, py - 5); ctx.lineTo(px, py + 5);
        ctx.stroke();
      }
    }
  }, [grid, width, height, resolution, originX, originY, selectedTarget]);

  function handleCanvasClick(event: MouseEvent<HTMLCanvasElement>) {
    if (!onSelectTarget || width <= 0 || height <= 0) return;
    const rect = event.currentTarget.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const px = ((event.clientX - rect.left) / rect.width) * width;
    const py = ((event.clientY - rect.top) / rect.height) * height;
    const x_m = originX + px * resolution;
    const y_m = originY + (height - 1 - py) * resolution;
    onSelectTarget(Math.round(x_m * 100) / 100, Math.round(y_m * 100) / 100);
  }

  return <section className="card map-cache-card" style={{marginTop: 0}}>
    <div className="section-heading">
      <div>
        <p className="eyebrow">VENDOR V1 OCCUPANCY GRID</p>
        <h2>Map</h2>
      </div>
      <button className="quiet" disabled={busy} onClick={onReadMap}>Read ROS map</button>
    </div>
    <p className="caption">
      Click the map to set target / drop coordinates{selectedTarget ? ` (${selectedTarget.x.toFixed(2)}, ${selectedTarget.y.toFixed(2)} m)` : ''}.
    </p>
    {grid && width > 0 && height > 0 && (
      <div className="map-panel" style={{marginTop: 12, paddingTop: 12}}>
        <figure className="map-figure">
          <div className="map-image-wrap">
            <canvas
              ref={mapCanvasRef}
              onClick={handleCanvasClick}
              style={{ display: 'block', width: '100%', maxHeight: '340px', imageRendering: 'pixelated', cursor: onSelectTarget ? 'crosshair' : 'default' }}
            />
          </div>
          <figcaption className="caption">
            Grid {String(width)}×{String(height)} cells @ {resolution.toFixed(2)} m/cell
          </figcaption>
        </figure>
      </div>
    )}
  </section>;
}
