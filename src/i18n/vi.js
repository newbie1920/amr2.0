/**
 * AMR 2.0 — Bản dịch tiếng Việt
 */
const vi = {
  // App
  appName: 'AMR 2.0 — Quản Lý Robot Vận Chuyển',
  appTitle: 'Trung Tâm Điều Khiển AMR',

  // Navigation
  nav: {
    dashboard: 'Bảng điều khiển',
    map: 'Bản đồ kho',
    tasks: 'Nhiệm vụ',
    robots: 'Quản lý xe',
    settings: 'Cài đặt',
  },

  // Connection
  connection: {
    title: 'Kết Nối Robot',
    addRobot: 'Thêm Robot',
    ipAddress: 'Địa chỉ IP',
    ipPlaceholder: 'VD: 192.168.1.100',
    port: 'Cổng',
    robotName: 'Tên robot',
    namePlaceholder: 'VD: Robot 1',
    connect: 'Kết nối',
    disconnect: 'Ngắt kết nối',
    connected: 'Đã kết nối',
    disconnected: 'Mất kết nối',
    connecting: 'Đang kết nối...',
    remove: 'Xóa',
    status: 'Trạng thái',
  },

  // Warehouse
  warehouse: {
    title: 'Bản Đồ Kho Xưởng',
    shelf: 'Kệ',
    level: 'Tầng',
    slot: 'Ô',
    gate: 'Cổng',
    importGate: 'Cổng Nhập',
    exportGate: 'Cổng Xuất',
    chargingStation: 'Trụ Sạc',
    empty: 'Trống',
    occupied: 'Có hàng',
    dimensions: 'Kích thước: 10m × 10m',
  },

  // Tasks
  task: {
    title: 'Quản Lý Nhiệm Vụ',
    importGoods: 'Nhập Hàng',
    exportGoods: 'Xuất Hàng',
    newTask: 'Tạo Nhiệm Vụ Mới',
    selectOrder: 'Chọn đơn hàng',
    selectShelf: 'Chọn kệ',
    selectLevel: 'Chọn tầng',
    selectSlot: 'Chọn ô',
    assignRobot: 'Giao cho robot',
    autoAssign: 'Tự động chọn xe',
    start: 'Bắt đầu',
    cancel: 'Hủy',
    status: {
      pending: 'Chờ xử lý',
      inProgress: 'Đang thực hiện',
      completed: 'Hoàn thành',
      failed: 'Thất bại',
      cancelled: 'Đã hủy',
    },
    steps: {
      goToGate: 'Đi đến cổng',
      pickUp: 'Bốc hàng',
      goToShelf: 'Đi đến kệ',
      placeItem: 'Đặt hàng',
      goToCharge: 'Đi sạc pin',
      returning: 'Quay về',
      rotating: 'Xoay góc',
    },
  },

  // Robot
  robot: {
    title: 'Thông Tin Robot',
    name: 'Tên',
    ip: 'Địa chỉ IP',
    status: 'Trạng thái',
    battery: 'Pin',
    position: 'Vị trí',
    heading: 'Hướng',
    speed: 'Tốc độ',
    distance: 'Quãng đường',
    currentTask: 'Nhiệm vụ hiện tại',
    noTask: 'Không có nhiệm vụ',
    states: {
      idle: 'Rảnh',
      moving: 'Đang di chuyển',
      charging: 'Đang sạc',
      working: 'Đang làm việc',
      error: 'Lỗi',
      offline: 'Ngoại tuyến',
      lowBattery: 'Pin yếu',
    },
    controls: {
      forward: 'Tiến',
      backward: 'Lùi',
      left: 'Trái',
      right: 'Phải',
      stop: 'Dừng',
      resetOdom: 'Đặt lại vị trí',
    },
  },

  // Status bar
  statusBar: {
    robotsOnline: 'Robot trực tuyến',
    robotsOffline: 'Robot ngoại tuyến',
    activeTasks: 'Nhiệm vụ đang chạy',
    completedTasks: 'Đã hoàn thành',
    warnings: 'Cảnh báo',
  },

  // Common
  common: {
    save: 'Lưu',
    cancel: 'Hủy',
    delete: 'Xóa',
    edit: 'Sửa',
    confirm: 'Xác nhận',
    close: 'Đóng',
    search: 'Tìm kiếm',
    noData: 'Không có dữ liệu',
    loading: 'Đang tải...',
    error: 'Lỗi',
    success: 'Thành công',
    warning: 'Cảnh báo',
    meters: 'm',
    degrees: '°',
    percent: '%',
  },

  // Alerts
  alerts: {
    lowBattery: (name, pct) => `⚠️ ${name} pin yếu (${pct}%)! Đang đi sạc...`,
    connectionLost: (name) => `❌ Mất kết nối ${name}!`,
    taskCompleted: (taskId) => `✅ Nhiệm vụ #${taskId} hoàn thành!`,
    taskFailed: (taskId, reason) => `❌ Nhiệm vụ #${taskId} thất bại: ${reason}`,
    robotCollision: (r1, r2) => `⚠️ Nguy cơ va chạm giữa ${r1} và ${r2}!`,
  },
};

export default vi;
