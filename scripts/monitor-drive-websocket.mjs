import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const HEADER = 'sequence,time_ms,dt_ms,left_mask,right_mask,acceleration_mask,left_raw_rpm,right_raw_rpm,left_rpm,right_rpm,left_accel_rpm_s,right_accel_rpm_s,left_actual_mV,right_actual_mV,LF_rpm,LM_rpm,LBD_rpm,LBU_rpm,RF_rpm,RM_rpm,RBD_rpm,RBU_rpm,dropped,command_valid,enabled,left_command_mV,right_command_mV,mode,target_rpm';
const BASE = 'ws://127.0.0.1:7071/vexrobotics.vexcode';
export const MOTOR_HEADER = 'sequence,time_ms,LM_deg,LM_current_A,LF_deg,LF_current_A,diagnostic_mask';
export const HEADING_HEADER = 'sequence,heading_time_ms,heading_deg,target_deg,error_deg,correction_mV,locked,ready,active';
export const VELOCITY_HEADER = 'sequence,velocity_time_ms,left_target_rpm,right_target_rpm,left_error_rpm,right_error_rpm,left_correction_mV,right_correction_mV,active';
export const PROFILE_HEADER = 'sequence,motion_time_ms,elapsed_ms,reference_mm,reference_rpm,distance_mm,target_rpm,remaining_mm,duration_ms,running,phase';
export const QUALITY_HEADER = 'sequence,time_ms,left_rejected_mask,right_rejected_mask,left_encoder_valid,right_encoder_valid,left_encoder_rpm,right_encoder_rpm,left_sdk_valid,right_sdk_valid';

// 串口数据可跨越多个 WebSocket 消息，必须拼成完整行后再解析。
export class DriveLines {
  pending = '';
  malformed = 0;
  other = 0;
  diagnostics = 0;
  headings = 0;
  velocities = 0;
  profiles = 0;
  qualities = 0;
  constructor(onSample, onDiagnostic = () => {}, onHeading = () => {}, onVelocity = () => {}, onProfile = () => {}, onQuality = () => {}) {
    this.onSample = onSample; this.onDiagnostic = onDiagnostic; this.onHeading = onHeading;
    this.onVelocity = onVelocity;
    this.onProfile = onProfile;
    this.onQuality = onQuality;
  }
  push(text) {
    this.pending += text;
    let end;
    while ((end = this.pending.indexOf('\n')) !== -1) {
      const line = this.pending.slice(0, end).replace(/\x1b\[[0-9;]*m/g, '').trim();
      this.pending = this.pending.slice(end + 1);
      const record = line.startsWith('#web#') ? line.slice(5) : line;
      const legacy = record.startsWith('DSV,');
      const diagnostic = record.startsWith('DLM,');
      const heading = record.startsWith('DHD,');
      const velocity = record.startsWith('DVL,');
      const profile = record.startsWith('DTP,');
      const quality = record.startsWith('DSQ,');
      if (!legacy && !diagnostic && !heading && !velocity && !profile && !quality && !record.startsWith('DS2,')) { this.other++; continue; }
      const fields = record.slice(4).split(',');
      if (fields.length !== (quality ? 10 : (profile ? 11 : (heading || velocity ? 9 : (diagnostic ? 7 : (legacy ? 27 : 29))))) || fields.some(x => !x.trim() || !Number.isFinite(Number(x)))) {
        this.malformed++;
        continue;
      }
      if (diagnostic) { this.diagnostics++; this.onDiagnostic(fields); continue; }
      if (heading) { this.headings++; this.onHeading(fields); continue; }
      if (velocity) { this.velocities++; this.onVelocity(fields); continue; }
      if (profile) { this.profiles++; this.onProfile(fields); continue; }
      if (quality) { this.qualities++; this.onQuality(fields); continue; }
      // 旧日志没有模式和目标转速，标为未知模式 -1，不能反推出目标转速。
      this.onSample(legacy ? [...fields, '-1', '0'] : fields);
    }
    // 异常输出不能无限占用内存；丢弃过长且尚未结束的行。
    if (this.pending.length > 65536) { this.pending = ''; this.malformed++; }
  }
}

function selectedDevice() {
  return new Promise((resolve, reject) => {
    const query = new WebSocket(`${BASE}/query`);
    query.binaryType = 'arraybuffer';
    const timeout = setTimeout(() => { query.close(); reject(new Error('查询 VS Code 超时')); }, 5000);
    query.onopen = () => query.send(JSON.stringify({ command: 'Selected Device' }));
    query.onmessage = event => {
      try {
        const text = typeof event.data === 'string' ? event.data : new TextDecoder().decode(event.data);
        const device = JSON.parse(text).result;
        if (!device?.instanceId) throw new Error('VEX 插件未选中设备');
        clearTimeout(timeout); query.close(); resolve(device);
      } catch (error) { clearTimeout(timeout); query.close(); reject(error); }
    };
    query.onerror = () => { clearTimeout(timeout); reject(new Error('无法连接本机 VEX WebSocket；检查 VS Code 和工作区设置')); };
    query.onclose = () => { clearTimeout(timeout); reject(new Error('设备查询连接已关闭')); };
  });
}

export async function capture(seconds) {
  if (typeof WebSocket !== 'function') throw new Error('请使用 Node.js 22 或更新版本');
  const device = await selectedDevice();
  const directory = fileURLToPath(new URL('../build/', import.meta.url));
  fs.mkdirSync(directory, { recursive: true });
  const stem = path.join(directory, `drive-websocket-${Date.now()}`);
  // wx 防止覆盖已有的实测数据。原始日志保留不完整行，便于定位传输问题。
  const csv = fs.openSync(`${stem}.csv`, 'wx');
  let raw, motors, headings, velocities, profiles, quality;
  try {
    raw = fs.openSync(`${stem}.log`, 'wx');
    motors = fs.openSync(`${stem}-motors.csv`, 'wx');
    fs.writeSync(motors, `${MOTOR_HEADER}\n`);
    headings = fs.openSync(`${stem}-heading.csv`, 'wx');
    fs.writeSync(headings, `${HEADING_HEADER}\n`);
    velocities = fs.openSync(`${stem}-velocity.csv`, 'wx');
    fs.writeSync(velocities, `${VELOCITY_HEADER}\n`);
    profiles = fs.openSync(`${stem}-profile.csv`, 'wx');
    fs.writeSync(profiles, `${PROFILE_HEADER}\n`);
    quality = fs.openSync(`${stem}-quality.csv`, 'wx');
    fs.writeSync(quality, `${QUALITY_HEADER}\n`);
  } catch (error) {
    for (const fd of [csv, raw, motors, headings, velocities, profiles, quality]) if (fd !== undefined) fs.closeSync(fd);
    throw error;
  }
  let samples = 0, bytes = 0, latest, latestQuality, connected = false;
  const parser = new DriveLines(fields => {
    fs.writeSync(csv, `${fields.join(',')}\n`);
    samples++; latest = fields;
  }, fields => fs.writeSync(motors, `${fields.join(',')}\n`),
  fields => fs.writeSync(headings, `${fields.join(',')}\n`),
  fields => fs.writeSync(velocities, `${fields.join(',')}\n`),
  fields => fs.writeSync(profiles, `${fields.join(',')}\n`),
  fields => { fs.writeSync(quality, `${fields.join(',')}\n`); latestQuality = fields; });
  fs.writeSync(csv, `${HEADER}\n`);
  console.log(`只读采集 ${seconds}s，设备 ${device.platform} ${device.device}，VS Code ID=${device.instanceId}`);
  console.log(`CSV: ${stem}.csv`);
  console.log(`LM/LF 角度和电流: ${stem}-motors.csv`);
  console.log(`航向和修正电压: ${stem}-heading.csv`);
  console.log(`速度闭环目标、误差和补偿: ${stem}-velocity.csv`);
  console.log(`梯形参考、距离和阶段: ${stem}-profile.csv`);
  console.log(`SDK 异常掩码和编码器差分速度: ${stem}-quality.csv`);
  try {
    await new Promise((resolve, reject) => {
      const ws = new WebSocket(`${BASE}/device?id=${encodeURIComponent(device.instanceId)}`);
      ws.binaryType = 'arraybuffer';
      const decoder = new TextDecoder();
      let endTimer;
      let finished = false;
      const finish = error => {
        if (finished) return;
        finished = true;
        clearTimeout(connectTimer); clearTimeout(endTimer); clearInterval(status);
        process.off('SIGINT', stop);
        ws.close();
        error ? reject(error) : resolve();
      };
      const stop = () => finish();
      const connectTimer = setTimeout(() => finish(new Error('连接设备数据通道超时')), 5000);
      const status = setInterval(() => {
        const encoder = latest?.[27] === '6';
        const aligned = latestQuality && latestQuality[0] === latest?.[0];
        const left = encoder ? (aligned && latestQuality[4] === '1' ? latestQuality[6] : '--') : latest?.[8];
        const right = encoder ? (aligned && latestQuality[5] === '1' ? latestQuality[7] : '--') : latest?.[9];
        const values = latest ? `L=${left} R=${right} rpm（${encoder ? '编码器差分' : 'SDK滤波'}），在线掩码=${latest[3]}/${latest[4]}，模式=${['手控','固定电压','前馈','A/B测试','前馈+航向','前馈+航向+速度PID','TP梯形规划','比赛自动'][latest[27]] ?? '旧版'} T=${latest[28]}rpm` : '尚无速度采样；请确认遥控器 LOG:ON（Y 切换）';
        console.log(`${values}；样本=${samples}，电机诊断=${parser.diagnostics}，航向=${parser.headings}，速度闭环=${parser.velocities}，规划=${parser.profiles}，质量=${parser.qualities}，接收=${bytes} 字节`);
      }, 5000);
      process.once('SIGINT', stop);
      ws.onopen = () => {
        connected = true; clearTimeout(connectTimer);
        console.log('数据通道已连接；按遥控器 Y 开始回传，再按一次停止。脚本不会向主控发送指令。');
        endTimer = setTimeout(() => finish(), seconds * 1000);
      };
      ws.onmessage = event => {
        if (finished) return;
        try {
          const data = typeof event.data === 'string' ? event.data : new Uint8Array(event.data);
          fs.writeSync(raw, data);
          bytes += typeof data === 'string' ? Buffer.byteLength(data) : data.length;
          parser.push(typeof data === 'string' ? data : decoder.decode(data, { stream: true }));
        } catch (error) { finish(error); }
      };
      ws.onerror = () => finish(new Error('设备 WebSocket 连接失败'));
      ws.onclose = () => finish(new Error('设备数据通道提前关闭；检查 VS Code 设备选择和连接'));
    });
  } finally {
    for (const fd of [csv, raw, motors, headings, velocities, profiles, quality]) fs.closeSync(fd);
    console.log(`采集结束：连接=${connected}，字节=${bytes}，有效样本=${samples}，电机诊断=${parser.diagnostics}，航向=${parser.headings}，速度闭环=${parser.velocities}，规划=${parser.profiles}，质量=${parser.qualities}，坏行=${parser.malformed}，其他行=${parser.other}，末尾未完成字符=${parser.pending.length}`);
  }
  if (!samples) throw new Error(bytes ? '收到输出，但没有完整速度数据；请确认运行的是包含底盘采样的 Bartmoss' : '没有收到样本；请确认 Bartmoss 运行中、LOG:ON 和无线连接正常');
  if (!parser.diagnostics) console.log('未收到 LM/LF 角度、电流记录；仅有速度数据，不能完成电机交叉诊断。');
  return { samples, bytes, diagnostics: parser.diagnostics };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const seconds = Number(process.argv[2] ?? 30);
  if (!Number.isInteger(seconds) || seconds < 1 || seconds > 600) {
    console.error('用法：node scripts/monitor-drive-websocket.mjs [采集秒数 1–600]');
    process.exitCode = 1;
  } else capture(seconds).catch(error => { console.error(error.message); process.exitCode = 1; });
}
