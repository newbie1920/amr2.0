import fs from 'node:fs';
import path from 'node:path';

const DEFAULT_INPUT = 'docs/report_plots/sample_telemetry.csv';
const DEFAULT_OUTPUT = 'docs/report_plots/output';
const WIDTH = 960;
const HEIGHT = 540;
const MARGIN = { top: 42, right: 34, bottom: 68, left: 78 };

function parseArgs(argv) {
  const args = {
    input: DEFAULT_INPUT,
    output: DEFAULT_OUTPUT,
    map: null,
  };

  for (let i = 2; i < argv.length; i += 1) {
    const arg = argv[i];
    const next = argv[i + 1];
    if ((arg === '--input' || arg === '-i') && next) {
      args.input = next;
      i += 1;
    } else if ((arg === '--output' || arg === '-o') && next) {
      args.output = next;
      i += 1;
    } else if (arg === '--map' && next) {
      args.map = next;
      i += 1;
    } else if (arg === '--help' || arg === '-h') {
      printHelp();
      process.exit(0);
    }
  }

  return args;
}

function printHelp() {
  console.log(`Usage:
  npm.cmd run report:plots
  npm.cmd run report:plots -- --input logs/telemetry.csv --output docs/report_plots/output
  npm.cmd run report:plots -- --input logs/telemetry.csv --map exported_map.json

CSV columns:
  required: time or timestamp, x, y
  optional: targetX, targetY, headingDeg/heading, headingRad/theta, linearVel/vx, angularVel/wz, battery/batt
`);
}

function parseCsv(filePath) {
  const text = fs.readFileSync(filePath, 'utf8').replace(/^\uFEFF/, '').trim();
  if (!text) throw new Error(`CSV is empty: ${filePath}`);

  const lines = text.split(/\r?\n/).filter(Boolean);
  const headers = splitCsvLine(lines[0]).map((h) => h.trim());

  return lines.slice(1).map((line, rowIndex) => {
    const values = splitCsvLine(line);
    const row = {};
    headers.forEach((header, index) => {
      row[header] = parseCell(values[index]);
    });
    row.__row = rowIndex + 2;
    return row;
  });
}

function splitCsvLine(line) {
  const cells = [];
  let current = '';
  let inQuotes = false;

  for (let i = 0; i < line.length; i += 1) {
    const ch = line[i];
    const next = line[i + 1];
    if (ch === '"' && inQuotes && next === '"') {
      current += '"';
      i += 1;
    } else if (ch === '"') {
      inQuotes = !inQuotes;
    } else if (ch === ',' && !inQuotes) {
      cells.push(current);
      current = '';
    } else {
      current += ch;
    }
  }
  cells.push(current);
  return cells;
}

function parseCell(value) {
  const trimmed = String(value ?? '').trim();
  if (trimmed === '') return null;
  const numeric = Number(trimmed);
  return Number.isFinite(numeric) ? numeric : trimmed;
}

function pick(row, names, fallback = null) {
  for (const name of names) {
    if (row[name] !== undefined && row[name] !== null && row[name] !== '') return row[name];
  }
  return fallback;
}

function normalizeRows(rows) {
  const normalized = rows.map((row) => {
    const time = Number(pick(row, ['time', 'timestamp', 't', 'seconds', 'sec']));
    const x = Number(pick(row, ['x', 'posX', 'robotX', 'slamMapX', 'odomX']));
    const y = Number(pick(row, ['y', 'posY', 'robotY', 'slamMapY', 'odomY']));
    const targetX = pick(row, ['targetX', 'goalX', 'refX', 'xRef']);
    const targetY = pick(row, ['targetY', 'goalY', 'refY', 'yRef']);
    const headingRad = pick(row, ['headingRad', 'theta', 'slamMapTheta', 'odomTheta']);
    const headingDeg = pick(row, ['headingDeg', 'heading', 'h']);
    const linearVel = pick(row, ['linearVel', 'vx', 'v', 'speed']);
    const angularVel = pick(row, ['angularVel', 'wz', 'w', 'omega']);
    const battery = pick(row, ['battery', 'batt', 'battV']);

    return {
      time,
      x,
      y,
      targetX: targetX == null ? null : Number(targetX),
      targetY: targetY == null ? null : Number(targetY),
      headingDeg: headingDeg == null && headingRad != null ? Number(headingRad) * 180 / Math.PI : Number(headingDeg),
      linearVel: linearVel == null ? null : Number(linearVel),
      angularVel: angularVel == null ? null : Number(angularVel),
      battery: battery == null ? null : Number(battery),
      sourceRow: row.__row,
    };
  });

  const valid = normalized.filter((row) => (
    Number.isFinite(row.time) && Number.isFinite(row.x) && Number.isFinite(row.y)
  ));

  if (valid.length < 2) {
    throw new Error('Need at least two valid rows with time, x, y columns.');
  }

  return valid;
}

function extent(values, paddingRatio = 0.06) {
  const finite = values.filter(Number.isFinite);
  let min = Math.min(...finite);
  let max = Math.max(...finite);
  if (min === max) {
    min -= 1;
    max += 1;
  }
  const padding = (max - min) * paddingRatio;
  return [min - padding, max + padding];
}

function scaleLinear(domain, range) {
  const [d0, d1] = domain;
  const [r0, r1] = range;
  return (value) => r0 + ((value - d0) / (d1 - d0)) * (r1 - r0);
}

function linePath(points, xScale, yScale) {
  return points
    .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y))
    .map((point, index) => `${index === 0 ? 'M' : 'L'} ${xScale(point.x).toFixed(2)} ${yScale(point.y).toFixed(2)}`)
    .join(' ');
}

function escapeXml(value) {
  return String(value)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;');
}

function ticks([min, max], count = 6) {
  const step = (max - min) / (count - 1);
  return Array.from({ length: count }, (_, index) => min + step * index);
}

function renderSvg({ title, xLabel, yLabel, xDomain, yDomain, series, annotations = [] }) {
  const plotW = WIDTH - MARGIN.left - MARGIN.right;
  const plotH = HEIGHT - MARGIN.top - MARGIN.bottom;
  const xScale = scaleLinear(xDomain, [MARGIN.left, MARGIN.left + plotW]);
  const yScale = scaleLinear(yDomain, [MARGIN.top + plotH, MARGIN.top]);

  const xTicks = ticks(xDomain);
  const yTicks = ticks(yDomain);

  const grid = [
    ...xTicks.map((tick) => `<line x1="${xScale(tick).toFixed(2)}" y1="${MARGIN.top}" x2="${xScale(tick).toFixed(2)}" y2="${MARGIN.top + plotH}" class="grid" />`),
    ...yTicks.map((tick) => `<line x1="${MARGIN.left}" y1="${yScale(tick).toFixed(2)}" x2="${MARGIN.left + plotW}" y2="${yScale(tick).toFixed(2)}" class="grid" />`),
  ].join('\n');

  const axes = `
    <line x1="${MARGIN.left}" y1="${MARGIN.top + plotH}" x2="${MARGIN.left + plotW}" y2="${MARGIN.top + plotH}" class="axis" />
    <line x1="${MARGIN.left}" y1="${MARGIN.top}" x2="${MARGIN.left}" y2="${MARGIN.top + plotH}" class="axis" />
    ${xTicks.map((tick) => `<text x="${xScale(tick).toFixed(2)}" y="${MARGIN.top + plotH + 28}" class="tick" text-anchor="middle">${formatNumber(tick)}</text>`).join('\n')}
    ${yTicks.map((tick) => `<text x="${MARGIN.left - 14}" y="${(yScale(tick) + 5).toFixed(2)}" class="tick" text-anchor="end">${formatNumber(tick)}</text>`).join('\n')}
  `;

  const paths = series.map((item) => {
    const pathData = linePath(item.points, xScale, yScale);
    const strokeDasharray = item.dashed ? ' stroke-dasharray="8 7"' : '';
    return `<path d="${pathData}" fill="none" stroke="${item.color}" stroke-width="${item.width ?? 3}"${strokeDasharray} class="series" />`;
  }).join('\n');

  const legend = series.map((item, index) => {
    const x = MARGIN.left + index * 210;
    const y = HEIGHT - 22;
    const dash = item.dashed ? ' stroke-dasharray="8 7"' : '';
    return `
      <line x1="${x}" y1="${y - 5}" x2="${x + 34}" y2="${y - 5}" stroke="${item.color}" stroke-width="4"${dash} />
      <text x="${x + 44}" y="${y}" class="legend">${escapeXml(item.name)}</text>
    `;
  }).join('\n');

  return `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="${WIDTH}" height="${HEIGHT}" viewBox="0 0 ${WIDTH} ${HEIGHT}">
  <style>
    .bg { fill: #fbfbf8; }
    .title { font: 700 24px Arial, sans-serif; fill: #172026; }
    .label { font: 600 15px Arial, sans-serif; fill: #38434f; }
    .tick, .legend { font: 13px Arial, sans-serif; fill: #47515c; }
    .grid { stroke: #d8ded8; stroke-width: 1; }
    .axis { stroke: #24313a; stroke-width: 1.5; }
    .series { stroke-linecap: round; stroke-linejoin: round; }
    .note { font: 12px Arial, sans-serif; fill: #5a646e; }
  </style>
  <rect class="bg" width="100%" height="100%" />
  <text x="${MARGIN.left}" y="30" class="title">${escapeXml(title)}</text>
  ${grid}
  ${axes}
  ${paths}
  ${annotations.join('\n')}
  <text x="${MARGIN.left + (WIDTH - MARGIN.left - MARGIN.right) / 2}" y="${HEIGHT - 38}" class="label" text-anchor="middle">${escapeXml(xLabel)}</text>
  <text x="22" y="${MARGIN.top + (HEIGHT - MARGIN.top - MARGIN.bottom) / 2}" class="label" text-anchor="middle" transform="rotate(-90 22 ${MARGIN.top + (HEIGHT - MARGIN.top - MARGIN.bottom) / 2})">${escapeXml(yLabel)}</text>
  ${legend}
</svg>
`;
}

function formatNumber(value) {
  const abs = Math.abs(value);
  if (abs >= 100) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  return value.toFixed(2);
}

function writeSvg(filePath, svg) {
  fs.writeFileSync(filePath, svg, 'utf8');
}

function computePositionError(rows) {
  return rows
    .filter((row) => Number.isFinite(row.targetX) && Number.isFinite(row.targetY))
    .map((row) => ({
      x: row.time,
      y: Math.hypot(row.targetX - row.x, row.targetY - row.y),
    }));
}

function generatePlots(rows, outputDir) {
  fs.mkdirSync(outputDir, { recursive: true });

  const timeDomain = extent(rows.map((row) => row.time), 0.01);
  const trajectoryDomainX = extent([
    ...rows.map((row) => row.x),
    ...rows.map((row) => row.targetX).filter(Number.isFinite),
  ]);
  const trajectoryDomainY = extent([
    ...rows.map((row) => row.y),
    ...rows.map((row) => row.targetY).filter(Number.isFinite),
  ]);

  const actualTrajectory = rows.map((row) => ({ x: row.x, y: row.y }));
  const targetTrajectory = rows
    .filter((row) => Number.isFinite(row.targetX) && Number.isFinite(row.targetY))
    .map((row) => ({ x: row.targetX, y: row.targetY }));

  writeSvg(path.join(outputDir, '01_trajectory_xy.svg'), renderSvg({
    title: 'Quy dao robot tren mat phang X-Y',
    xLabel: 'X (m)',
    yLabel: 'Y (m)',
    xDomain: trajectoryDomainX,
    yDomain: trajectoryDomainY,
    series: [
      { name: 'Duong di thuc te', color: '#16697a', points: actualTrajectory, width: 4 },
      ...(targetTrajectory.length ? [{ name: 'Duong muc tieu', color: '#db6400', points: targetTrajectory, dashed: true, width: 3 }] : []),
    ],
  }));

  const errorPoints = computePositionError(rows);
  if (errorPoints.length) {
    writeSvg(path.join(outputDir, '02_position_error.svg'), renderSvg({
      title: 'Sai so vi tri theo thoi gian',
      xLabel: 'Thoi gian (s)',
      yLabel: 'Sai so vi tri (m)',
      xDomain: timeDomain,
      yDomain: extent(errorPoints.map((point) => point.y), 0.12),
      series: [{ name: 'Sai so vi tri', color: '#b02a30', points: errorPoints, width: 4 }],
    }));
  }

  const headingPoints = rows
    .filter((row) => Number.isFinite(row.headingDeg))
    .map((row) => ({ x: row.time, y: row.headingDeg }));
  if (headingPoints.length) {
    writeSvg(path.join(outputDir, '03_heading.svg'), renderSvg({
      title: 'Goc huong robot theo thoi gian',
      xLabel: 'Thoi gian (s)',
      yLabel: 'Heading (deg)',
      xDomain: timeDomain,
      yDomain: extent(headingPoints.map((point) => point.y), 0.08),
      series: [{ name: 'Heading', color: '#4c6b35', points: headingPoints, width: 4 }],
    }));
  }

  const velocitySeries = [];
  const linearPoints = rows
    .filter((row) => Number.isFinite(row.linearVel))
    .map((row) => ({ x: row.time, y: row.linearVel }));
  const angularPoints = rows
    .filter((row) => Number.isFinite(row.angularVel))
    .map((row) => ({ x: row.time, y: row.angularVel }));
  if (linearPoints.length) velocitySeries.push({ name: 'Van toc tuyen tinh (m/s)', color: '#1f5f99', points: linearPoints, width: 4 });
  if (angularPoints.length) velocitySeries.push({ name: 'Van toc goc (rad/s)', color: '#8a4f7d', points: angularPoints, dashed: true, width: 3 });
  if (velocitySeries.length) {
    writeSvg(path.join(outputDir, '04_velocity.svg'), renderSvg({
      title: 'Van toc robot theo thoi gian',
      xLabel: 'Thoi gian (s)',
      yLabel: 'Gia tri van toc',
      xDomain: timeDomain,
      yDomain: extent(velocitySeries.flatMap((item) => item.points.map((point) => point.y)), 0.12),
      series: velocitySeries,
    }));
  }

  const batteryPoints = rows
    .filter((row) => Number.isFinite(row.battery))
    .map((row) => ({ x: row.time, y: row.battery }));
  if (batteryPoints.length) {
    writeSvg(path.join(outputDir, '05_battery.svg'), renderSvg({
      title: 'Pin robot theo thoi gian',
      xLabel: 'Thoi gian (s)',
      yLabel: 'Pin / dien ap',
      xDomain: timeDomain,
      yDomain: extent(batteryPoints.map((point) => point.y), 0.08),
      series: [{ name: 'Battery', color: '#7b5d20', points: batteryPoints, width: 4 }],
    }));
  }

  writeSummary(rows, errorPoints, outputDir);
}

function writeSummary(rows, errorPoints, outputDir) {
  const duration = rows.at(-1).time - rows[0].time;
  const distance = rows.slice(1).reduce((sum, row, index) => {
    const prev = rows[index];
    return sum + Math.hypot(row.x - prev.x, row.y - prev.y);
  }, 0);
  const maxError = errorPoints.length ? Math.max(...errorPoints.map((point) => point.y)) : null;
  const meanError = errorPoints.length ? errorPoints.reduce((sum, point) => sum + point.y, 0) / errorPoints.length : null;

  const lines = [
    '# Report Plot Metrics',
    '',
    '| Metric | Value |',
    '| --- | ---: |',
    `| Samples | ${rows.length} |`,
    `| Duration | ${duration.toFixed(2)} s |`,
    `| Travel distance | ${distance.toFixed(2)} m |`,
  ];

  if (meanError != null && maxError != null) {
    lines.push(`| Mean position error | ${meanError.toFixed(3)} m |`);
    lines.push(`| Max position error | ${maxError.toFixed(3)} m |`);
  }

  fs.writeFileSync(path.join(outputDir, 'metrics_summary.md'), `${lines.join('\n')}\n`, 'utf8');
}

function generateMapPlot(mapPath, outputDir) {
  const raw = JSON.parse(fs.readFileSync(mapPath, 'utf8'));
  const map = raw.data ? raw : raw.data?.data;
  const width = Number(raw.width ?? map?.width);
  const height = Number(raw.height ?? map?.height);
  const cells = raw.cells ?? raw.data ?? raw.occupancy ?? null;
  if (!Number.isFinite(width) || !Number.isFinite(height) || !Array.isArray(cells)) {
    console.warn(`Skipping map plot: ${mapPath} is not a simple width/height/cell-array JSON export.`);
    return;
  }

  const cellSize = Math.max(2, Math.floor(760 / Math.max(width, height)));
  const svgW = width * cellSize;
  const svgH = height * cellSize;
  const rects = [];
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const value = Number(cells[y * width + x]);
      const fill = value > 50 ? '#24313a' : value < 20 ? '#f7f7f1' : '#c7cbc7';
      rects.push(`<rect x="${x * cellSize}" y="${y * cellSize}" width="${cellSize}" height="${cellSize}" fill="${fill}" />`);
    }
  }

  fs.writeFileSync(path.join(outputDir, '06_occupancy_map.svg'), `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="${svgW}" height="${svgH}" viewBox="0 0 ${svgW} ${svgH}">
${rects.join('\n')}
</svg>
`, 'utf8');
}

function main() {
  const args = parseArgs(process.argv);
  const rows = normalizeRows(parseCsv(args.input));
  generatePlots(rows, args.output);
  if (args.map) generateMapPlot(args.map, args.output);
  console.log(`Report plots generated in ${args.output}`);
}

main();
