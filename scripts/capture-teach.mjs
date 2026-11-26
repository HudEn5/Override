import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { BASE, selectedDevice } from './monitor-drive-websocket.mjs';

// 从第一个点开始，只接受同一批次连续的完整记录；再次按右键会开始新批次。
export class TeachPoints {
  pending = '';
  batch = 0;
  total = 0;
  rows = [];

  push(text) {
    this.pending += text;
    let end;
    while ((end = this.pending.indexOf('\n')) >= 0) {
      const line = this.pending.slice(0, end).trim().replace(/\x1b\[[0-9;]*m/g, '').replace(/^#web#/, '');
      this.pending = this.pending.slice(end + 1);
      const m = /^DWP,(\d+),(\d+),(\d+),(-?\d+\.\d),(-?\d+\.\d),(-?\d+\.\d)$/.exec(line);
      if (!m) continue;
      const [batch, index, total, x, y, heading] = m.slice(1).map(Number);
      if (!Number.isSafeInteger(batch) || batch < 1 || total < 1 || total > 64 ||
          index < 1 || index > total || ![x, y, heading].every(Number.isFinite)) continue;
      if (index === 1) {
        this.batch = batch;
        this.total = total;
        this.rows = [];
      }
      if (batch !== this.batch || total !== this.total || index !== this.rows.length + 1) continue;
      this.rows.push([index, x, y, heading]);
      if (index === total) return this.rows;
    }
    return null;
  }
}

async function captureTeach() {
  if (typeof WebSocket !== 'function') throw new Error('请使用 Node.js 22 或更新版本');
  const device = await selectedDevice();
  const points = new TeachPoints();
  const decoder = new TextDecoder();
  const rows = await new Promise((resolve, reject) => {
    const ws = new WebSocket(`${BASE}/device?id=${encodeURIComponent(device.instanceId)}`);
    ws.binaryType = 'arraybuffer';
    let timer = setTimeout(() => finish(null, '连接设备超时'), 5000);
    let done = false;
    function finish(value, error) {
      if (done) return;
      done = true;
      clearTimeout(timer);
      ws.close();
      if (error) reject(new Error(error));
      else resolve(value);
    }
    ws.onopen = () => {
      clearTimeout(timer);
      console.log('已连接。请在示教模式按方向键右导出；漏包可再按一次。');
      timer = setTimeout(() => finish(null, '未收到完整点表；点仍可在本次开机期间重发。'), 180000);
    };
    ws.onmessage = event => {
      const rows = points.push(typeof event.data === 'string' ? event.data : decoder.decode(event.data, { stream: true }));
      if (rows) finish(rows);
    };
    ws.onclose = () => finish(null, '设备数据通道已关闭');
    ws.onerror = () => finish(null, '设备数据通道连接失败');
  });
  const dir = fileURLToPath(new URL('../build/data/', import.meta.url));
  fs.mkdirSync(dir, { recursive: true });
  const target = path.join(dir, `teach-points-${Date.now()}.csv`);
  fs.writeFileSync(target, ['point,x_mm,y_mm,heading_deg', ...rows.map(r => r.join(',')), ''].join('\n'), { flag: 'wx' });
  console.log(`已保存 ${rows.length} 个点：${target}`);
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  captureTeach().catch(error => { console.error(error.message); process.exitCode = 1; });
}
