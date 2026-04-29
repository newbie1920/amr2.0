/**
 * AMR 2.0 — Map Exporter (ROS2 Compatible)
 * Xuất bản đồ OccupancyGrid ra định dạng .pgm và .yaml cho ROS2 Nav2 / map_server
 */

export function exportMapToROS2(mapData, mapName = 'warehouse_map') {
  if (!mapData) {
    console.error('[MapExport] Lỗi: Dữ liệu map không hợp lệ');
    return;
  }

  let { width, height, resolution, originX, originY, data, logOdds } = mapData;
  
  // Nếu truyền vào json (từ savedMaps) chứ không phải mapperInstance
  if (!data && logOdds) {
    try {
      const binary = atob(logOdds);
      const bytes = new Uint8Array(binary.length);
      for (let i = 0; i < binary.length; i++) {
        bytes[i] = binary.charCodeAt(i);
      }
      const logOddsArr = new Float32Array(bytes.buffer);
      data = new Uint8Array(width * height);
      for (let i = 0; i < logOddsArr.length; i++) {
        const p = 1.0 / (1.0 + Math.exp(-logOddsArr[i]));
        data[i] = Math.round(p * 255);
      }
    } catch (e) {
      console.error('Lỗi giải mã logOdds:', e);
      return;
    }
  }

  if (!data) {
    console.error('[MapExport] Lỗi: Không có mảng data');
    return;
  }

  
  // 1. Tạo file .pgm (P5 - Binary)
  // Trong ROS2: free=254, occupied=000, unknown=205
  // data của chúng ta: 0=free, 255=occupied, 128=unknown
  const pgmData = new Uint8Array(width * height);
  for (let i = 0; i < data.length; i++) {
    // Lật ngược y vì hình ảnh có hệ trục toạ độ bắt đầu từ góc trên cùng bên trái
    const x = i % width;
    const y = Math.floor(i / width);
    const flippedY = height - 1 - y;
    const flippedIdx = flippedY * width + x;

    const val = data[i];
    if (val === 128) {
      pgmData[flippedIdx] = 205; // unknown
    } else if (val > 128) {
      pgmData[flippedIdx] = 0;   // occupied
    } else {
      pgmData[flippedIdx] = 254; // free
    }
  }

  const header = `P5\n${width} ${height}\n255\n`;
  const headerBytes = new TextEncoder().encode(header);
  
  const pgmBlobData = new Uint8Array(headerBytes.length + pgmData.length);
  pgmBlobData.set(headerBytes);
  pgmBlobData.set(pgmData, headerBytes.length);
  
  const pgmBlob = new Blob([pgmBlobData], { type: 'image/x-portable-graymap' });
  const pgmUrl = URL.createObjectURL(pgmBlob);

  // 2. Tạo file .yaml
  // Lưu ý: do ta đã lật ảnh PGM, origin Y có thể giữ nguyên nếu ta map đúng, 
  // nhưng thường PGM coi (0,0) ở góc dưới cùng bên trái nếu hiển thị bằng Rviz.
  const yamlContent = `image: ${mapName}.pgm
mode: trinary
resolution: ${resolution}
origin: [${originX.toFixed(3)}, ${originY.toFixed(3)}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
`;

  const yamlBlob = new Blob([yamlContent], { type: 'text/yaml' });
  const yamlUrl = URL.createObjectURL(yamlBlob);

  // 3. Tải xuống tự động
  _downloadFile(pgmUrl, `${mapName}.pgm`);
  _downloadFile(yamlUrl, `${mapName}.yaml`);

  // Dọn dẹp URL Object
  setTimeout(() => {
    URL.revokeObjectURL(pgmUrl);
    URL.revokeObjectURL(yamlUrl);
  }, 1000);
}

function _downloadFile(url, filename) {
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}
