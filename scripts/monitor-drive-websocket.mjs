import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const HEADER = 'sequence,time_ms,dt_ms,left_mask,right_mask,acceleration_mask,left_raw_rpm,right_raw_rpm,left_rpm,right_rpm,left_accel_rpm_s,right_accel_rpm_s,left_actual_mV,right_actual_mV,LF_rpm,LM_rpm,LBD_rpm,LBU_rpm,RF_rpm,RM_rpm,RBD_rpm,RBU_rpm,dropped,command_valid,enabled,left_command_mV,right_command_mV,mode,target_rpm';
export const TUNING_HEADER = 'sequence,time_ms,run,stage,direction,state,elapsed_ms,distance_mm,remaining_mm,peak_rpm,peak_heading_deg,speed_rms_rpm,high_voltage_ms,at_speed_ms,qualified';
export const ROUTE_HEADER = 'sequence,time_ms,run,step,motion,active,finished,waiting,result';
export const PUSH_HEADER = 'sequence,time_ms,elapsed_ms,duration_ms,left_command_mV,right_command_mV,state,settling,left_rpm,right_rpm';
const BASE = 'ws://127.0.0.1:7071/vexrobotics.vexcode';
export const MOTOR_HEADER = 'sequence,time_ms,LM_deg,LM_current_A,LF_deg,LF_current_A,diagnostic_mask';
export const HEADING_HEADER = 'sequence,heading_time_ms,heading_deg,target_deg,error_deg,correction_mV,locked,ready,active';
export const VELOCITY_HEADER = 'sequence,velocity_time_ms,left_target_rpm,right_target_rpm,left_error_rpm,right_error_rpm,left_correction_mV,right_correction_mV,active';
export const PROFILE_HEADER = 'sequence,motion_time_ms,elapsed_ms,reference_mm,reference_rpm,distance_mm,target_rpm,remaining_mm,duration_ms,running,phase';
export const QUALITY_HEADER = 'sequence,time_ms,left_rejected_mask,right_rejected_mask,left_encoder_valid,right_encoder_valid,left_encoder_rpm,right_encoder_rpm,left_sdk_valid,right_sdk_valid';
export const TURN_HEADER = 'sequence,turn_time_ms,elapsed_ms,angle_deg,target_deg,error_deg,rate_deg_s,target_rpm,left_command_mV,right_command_mV,state';
export const TURN_PROFILE_HEADER = 'sequence,turn_time_ms,elapsed_ms,reference_deg,reference_rate_deg_s,duration_ms,peak_rate_deg_s,phase,tracking_error_deg';
export const POSE_HEADER = 'sequence,pose_time_ms,x_mm,y_mm,heading_deg,valid,needs_reset';
export const CORNER_HEADER = 'sequence,corner_time_ms,elapsed_ms,phase,state,distance_mm,remaining_mm,angle_deg,reference_deg,error_deg,rate_deg_s,center_rpm,left_target_rpm,right_target_rpm,left_command_mV,right_command_mV';

export const ROLLING_HEADER = 'sequence,rolling_time_ms,elapsed_ms,phase,state,distance_mm,segment_remaining_mm,angle_deg,reference_deg,error_deg,rate_deg_s,center_rpm,left_target_rpm,right_target_rpm,left_command_mV,right_command_mV,completed_turns,total_turns';

// 字段数正确仍可能是串口错位。检查整数、标志和枚举，不将明显破损的数值写入 CSV。
function validFields(kind, f) {
  const integer = (i, min = 0, max = Number.MAX_SAFE_INTEGER) =>
    /^-?\d+$/.test(f[i]) && Number.isSafeInteger(Number(f[i])) && Number(f[i]) >= min && Number(f[i]) <= max;
  const flags = (...indices) => indices.every(i => f[i] === '0' || f[i] === '1');
  const voltage = i => integer(i, -12000, 12000);
  if (!integer(0) || Number(f[1]) < 0) return false;
  switch (kind) {
    case 'DSV': case 'DS2':
      return Number(f[2]) >= 0 && integer(3, 0, 15) && integer(4, 0, 15) && integer(5, 0, 3) &&
        integer(22) && flags(23, 24) && [12, 13, 25, 26].every(voltage) &&
        (kind === 'DSV' || integer(27, 0, 8));
    case 'DLM': return integer(6, 0, 3);
    case 'DRA': return integer(2) && integer(3) && integer(4, 0, 5) && flags(5, 6, 7) &&
      integer(8, 0, 4) && !(f[5] === '1' && f[6] === '1') && (f[7] === '0' || f[5] === '1');
    case 'DPS': return Number(f[2]) >= 0 && Number(f[3]) >= 0 && Number(f[3]) <= 5000 &&
      integer(4, 0, 12000) && integer(5, 0, 12000) && integer(6, 0, 4) && flags(7) &&
      ((f[6] === '1' && f[7] === '0') || (f[4] === '0' && f[5] === '0'));
    case 'DHD': return flags(6, 7, 8);
    case 'DVL': return flags(8);
    case 'DSQ': return integer(2, 0, 15) && integer(3, 0, 15) && flags(4, 5, 8, 9);
    case 'DTP': return Number(f[2]) >= 0 && Number(f[8]) >= 0 && flags(9) && integer(10, 0, 7);
    case 'DTN': return Number(f[2]) >= 0 && voltage(8) && voltage(9) && integer(10, 0, 4);
    case 'DTA': return Number(f[2]) >= 0 && Number(f[5]) >= 0 && Number(f[6]) >= 0 && integer(7, 0, 4);
    case 'DPO': return flags(5,6) && !(f[5]==='1' && f[6]==='1');
    case 'DRL': return Number(f[2]) >= 0 && integer(3, 0, 4) && integer(4, 0, 4) && voltage(14) && voltage(15) &&
      integer(16, 0, 16) && integer(17, 0, 16) && Number(f[16]) <= Number(f[17]) &&
      (!['1','2'].includes(f[4]) || Number(f[17]) > 0);
    case 'DCR': return Number(f[2]) >= 0 && integer(3, 0, 4) && integer(4, 0, 4) && voltage(14) && voltage(15);
    case 'DHS': return integer(2, 1) && integer(3, 1, 4) && ['-1', '1'].includes(f[4]) &&
      integer(5, 0, 4) && [6, 9, 10, 11, 12, 13].every(i => Number(f[i]) >= 0) && flags(14) &&
      (f[14] === '0' || (f[5] === '2' && Number(f[13]) >= 300));
    default: return false;
  }
}

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
  turns = 0;
  corners = 0;
  rolling = 0;
  tuning = 0;
  routes = 0;
  pushes = 0;
  turnProfiles = 0;
  poses = 0;
  constructor(onSample, onDiagnostic = () => {}, onHeading = () => {}, onVelocity = () => {}, onProfile = () => {}, onQuality = () => {}, onTurn = () => {}, onCorner = () => {}, onTuning = () => {}, onRoute = () => {}, onPush = () => {}, onTurnProfile = () => {}, onPose = () => {}, onRolling = () => {}) {
    this.onSample = onSample; this.onDiagnostic = onDiagnostic; this.onHeading = onHeading;
    this.onVelocity = onVelocity;
    this.onProfile = onProfile;
    this.onQuality = onQuality;
    this.onTurn = onTurn;
    this.onCorner = onCorner; this.onTuning = onTuning;
    this.onRoute = onRoute;
    this.onPush = onPush;
    this.onTurnProfile = onTurnProfile;
    this.onPose = onPose;
    this.onRolling = onRolling;
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
      const turn = record.startsWith('DTN,');
      const corner = record.startsWith('DCR,');
      const rolling = record.startsWith('DRL,');
      const tuning = record.startsWith('DHS,');
      const route = record.startsWith('DRA,');
      const push = record.startsWith('DPS,');
      const turnProfile = record.startsWith('DTA,');
      const pose = record.startsWith('DPO,');
      if (!legacy && !diagnostic && !heading && !velocity && !profile && !quality && !turn && !corner && !rolling && !tuning && !route && !push && !turnProfile && !pose && !record.startsWith('DS2,')) { this.other++; continue; }
      const fields = record.slice(4).split(',');
      if (rolling) {
        if (fields.length !== 18 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DRL', fields)) this.malformed++;
        else { this.rolling++; this.onRolling(fields); }
        continue;
      }
      if (pose) {
        if (fields.length !== 7 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DPO', fields)) this.malformed++;
        else { this.poses++; this.onPose(fields); }
        continue;
      }
      if (turnProfile) {
        if (fields.length !== 9 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DTA', fields)) this.malformed++;
        else { this.turnProfiles++; this.onTurnProfile(fields); }
        continue;
      }
      if (push) {
        if (fields.length !== 10 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DPS', fields)) this.malformed++;
        else { this.pushes++; this.onPush(fields); }
        continue;
      }
      if (route) {
        if (fields.length !== 9 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DRA', fields)) this.malformed++;
        else { this.routes++; this.onRoute(fields); }
        continue;
      }
      if (tuning) {
        if (fields.length !== 15 || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields('DHS', fields)) this.malformed++;
        else { this.tuning++; this.onTuning(fields); }
        continue;
      }
      if (fields.length !== (corner ? 16 : (quality ? 10 : (profile || turn ? 11 : (heading || velocity ? 9 : (diagnostic ? 7 : (legacy ? 27 : 29)))))) || fields.some(x => !x.trim() || !Number.isFinite(Number(x))) || !validFields(record.slice(0, 3), fields)) {
        this.malformed++;
        continue;
      }
      if (diagnostic) { this.diagnostics++; this.onDiagnostic(fields); continue; }
      if (heading) { this.headings++; this.onHeading(fields); continue; }
      if (velocity) { this.velocities++; this.onVelocity(fields); continue; }
      if (profile) { this.profiles++; this.onProfile(fields); continue; }
      if (quality) { this.qualities++; this.onQuality(fields); continue; }
      if (turn) { this.turns++; this.onTurn(fields); continue; }
      if (corner) { this.corners++; this.onCorner(fields); continue; }
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
  const directory = fileURLToPath(new URL('../build/data/', import.meta.url));
  fs.mkdirSync(directory, { recursive: true });
  const stem = path.join(directory, `drive-websocket-${Date.now()}`);
  // wx 防止覆盖已有的实测数据。原始日志保留不完整行，便于定位传输问题。
  const csv = fs.openSync(`${stem}.csv`, 'wx');
  let raw, motors, headings, velocities, profiles, quality, turns, corners, tuning, routes, pushes, turnProfiles, poses, rolling;
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
    turns = fs.openSync(`${stem}-turn.csv`, 'wx');
    fs.writeSync(turns, `${TURN_HEADER}\n`);
    corners = fs.openSync(`${stem}-corner.csv`, 'wx');
    fs.writeSync(corners, `${CORNER_HEADER}\n`);
    rolling = fs.openSync(`${stem}-rolling.csv`, 'wx');
    fs.writeSync(rolling, `${ROLLING_HEADER}\n`);
    tuning = fs.openSync(`${stem}-tuning.csv`, 'wx');
    fs.writeSync(tuning, `${TUNING_HEADER}\n`);
    routes = fs.openSync(`${stem}-route.csv`, 'wx');
    fs.writeSync(routes, `${ROUTE_HEADER}\n`);
    pushes = fs.openSync(`${stem}-push.csv`, 'wx');
    fs.writeSync(pushes, `${PUSH_HEADER}\n`);
    turnProfiles = fs.openSync(`${stem}-turn-profile.csv`, 'wx');
    fs.writeSync(turnProfiles, `${TURN_PROFILE_HEADER}\n`);
    poses = fs.openSync(`${stem}-pose.csv`, 'wx');
    fs.writeSync(poses, `${POSE_HEADER}\n`);
  } catch (error) {
    for (const fd of [csv, raw, motors, headings, velocities, profiles, quality, turns, corners, tuning, routes, pushes, turnProfiles, poses, rolling]) if (fd !== undefined) fs.closeSync(fd);
    throw error;
  }
  let samples = 0, bytes = 0, latest, latestQuality, latestTuning, latestVelocity, latestCorner, latestRoute, connected = false;
  let latestRolling, latestPose, latestProfile, latestTurn, latestPush, latestKind = '', latestMotionSequence = -1;
  const parser = new DriveLines(fields => {
    fs.writeSync(csv, `${fields.join(',')}\n`);
    samples++; latest = fields;
  }, fields => fs.writeSync(motors, `${fields.join(',')}\n`),
  fields => fs.writeSync(headings, `${fields.join(',')}\n`),
  fields => { fs.writeSync(velocities, `${fields.join(',')}\n`); latestVelocity = fields; },
  fields => { fs.writeSync(profiles, `${fields.join(',')}\n`); latestProfile = fields; latestKind = 'straight'; latestMotionSequence = Number(fields[0]); },
  fields => { fs.writeSync(quality, `${fields.join(',')}\n`); latestQuality = fields; },
  fields => { fs.writeSync(turns, `${fields.join(',')}\n`); latestTurn = fields; latestKind = 'turn'; latestMotionSequence = Number(fields[0]); },
  fields => { fs.writeSync(corners, `${fields.join(',')}\n`); latestCorner = fields; latestKind = 'corner'; latestMotionSequence = Number(fields[0]); },
  fields => { fs.writeSync(tuning, `${fields.join(',')}\n`); latestTuning = fields; latestKind = 'tuning'; latestMotionSequence = Number(fields[0]); },
  fields => { fs.writeSync(routes, `${fields.join(',')}\n`); latestRoute = fields; },
  fields => { fs.writeSync(pushes, `${fields.join(',')}\n`); latestPush = fields; latestKind = 'push'; latestMotionSequence = Number(fields[0]); },
  fields => fs.writeSync(turnProfiles, `${fields.join(',')}\n`),
  fields => { fs.writeSync(poses, `${fields.join(',')}\n`); latestPose = fields; },
  fields => { fs.writeSync(rolling, `${fields.join(',')}\n`); latestRolling = fields; latestKind = 'rolling'; latestMotionSequence = Number(fields[0]); });
  fs.writeSync(csv, `${HEADER}\n`);
  console.log(`只读采集 ${seconds}s，设备 ${device.platform} ${device.device}，VS Code ID=${device.instanceId}`);
  console.log(`CSV: ${stem}.csv`);
  console.log(`LM/LF 角度和电流: ${stem}-motors.csv`);
  console.log(`航向和修正电压: ${stem}-heading.csv`);
  console.log(`速度闭环目标、误差和补偿: ${stem}-velocity.csv`);
  console.log(`梯形参考、距离和阶段: ${stem}-profile.csv`);
  console.log(`SDK 异常掩码和编码器差分速度: ${stem}-quality.csv`);
  console.log(`转向角度、角速度、轮速目标和状态: ${stem}-turn.csv`);
  console.log(`角速度规划参考、阶段及跟踪误差: ${stem}-turn-profile.csv`);
  console.log(`编码器与IMU位姿推算: ${stem}-pose.csv`);
  console.log(`连续圆角阶段、距离、航向和两侧目标: ${stem}-corner.csv`);
  console.log(`移动转向阶段、当前段余量和转向计数: ${stem}-rolling.csv`);
  console.log(`高速整定指标与终态: ${stem}-tuning.csv`);
  console.log(`自动路线步骤、等待与结果: ${stem}-route.csv`);
  console.log(`定时前推电压、计时和停稳: ${stem}-push.csv`);
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
        if (latestRoute && Number(latestRoute[0]) >= latestMotionSequence) {
          const a = latestRoute;
          console.log(`ROUTE run=${a[2]} step=${a[3]} ${a[7] === '1' ? 'WAIT' : ['READY','RUN','DONE','CANCEL','FAULT'][a[8]]}；路线=${parser.routes}，圆角=${parser.corners}，移动转向=${parser.rolling}，规划=${parser.profiles}，转向=${parser.turns}，坏行=${parser.malformed}，接收=${bytes} 字节`);
          return;
        }
        if (latestKind === 'rolling' && latestRolling) {
          const r = latestRolling;
          console.log(`ROLL ${['READY','RUN','DONE','CANCEL','FAULT'][r[4]]} 阶段=${r[3]} 转向=${r[16]}/${r[17]}；角度=${r[7]}° 误差=${r[9]}° 当前段剩余=${r[6]} mm`);
          return;
        }
        if (latestKind === 'corner' && latestCorner) {
          const c = latestCorner;
          console.log(`CNR ${['READY','RUN','DONE','CANCEL','FAULT'][c[4]]} 阶段=${c[3]}；角度=${c[7]}° 参考=${c[8]}° 误差=${c[9]}°，剩余=${c[6]} mm；圆角=${parser.corners}，移动转向=${parser.rolling}，坏行=${parser.malformed}，接收=${bytes} 字节`);
          return;
        }
        if (latestKind === 'tuning' && latestTuning) {
          const h = latestTuning;
          const speed = latestVelocity ? `L=${(Number(latestVelocity[2]) - Number(latestVelocity[4])).toFixed(1)} R=${(Number(latestVelocity[3]) - Number(latestVelocity[5])).toFixed(1)} rpm` : '等待速度曲线';
          console.log(`H${h[3]} run=${h[2]} ${h[4] === '1' ? '前进' : '后退'} ${['READY','RUN','DONE','CANCEL','FAULT'][h[5]]}；${speed}；峰值=${h[9]} rpm，航向峰值=${h[10]}°；汇总=${parser.tuning}，坏行=${parser.malformed}，接收=${bytes} 字节`);
          return;
        }
        if (latestKind === 'straight' && latestProfile) {
          const p = latestProfile;
          console.log(`STR ${['READY','ACCEL','CRUISE','DECEL','SETTLE','DONE','CANCEL','FAULT'][p[10]]}；距离=${p[5]} mm 剩余=${p[7]} mm 目标=${p[6]} rpm；规划=${parser.profiles}，坏行=${parser.malformed}`);
          return;
        }
        if (latestKind === 'turn' && latestTurn) {
          const t = latestTurn;
          console.log(`TURN ${['READY','RUN','DONE','CANCEL','FAULT'][t[10]]}；角度=${t[3]}° 目标=${t[4]}° 误差=${t[5]}°；转向=${parser.turns}，坏行=${parser.malformed}`);
          return;
        }
        if (latestKind === 'push' && latestPush) {
          const p = latestPush;
          console.log(`PUSH ${p[7] === '1' ? 'SETTLE' : ['READY','RUN','DONE','CANCEL','FAULT'][p[6]]}；${p[2]}/${p[3]} ms，L=${p[4]} R=${p[5]} mV；前推=${parser.pushes}，坏行=${parser.malformed}`);
          return;
        }
        if (!latest && latestPose) {
          const p = latestPose;
          console.log(`POSE X=${p[2]} Y=${p[3]} mm H=${p[4]}° ${p[6] === '1' ? 'RESET' : p[5] === '1' ? 'OK' : 'WAIT'}；位姿=${parser.poses}，坏行=${parser.malformed}`);
          return;
        }
        const encoder = ['3', '6', '7', '8'].includes(latest?.[27]);
        const aligned = latestQuality && latestQuality[0] === latest?.[0];
        const left = encoder ? (aligned && latestQuality[4] === '1' ? latestQuality[6] : '--') : latest?.[8];
        const right = encoder ? (aligned && latestQuality[5] === '1' ? latestQuality[7] : '--') : latest?.[9];
        const values = latest ? `L=${left} R=${right} rpm（${encoder ? '编码器差分' : 'SDK滤波'}），在线掩码=${latest[3]}/${latest[4]}，模式=${['手控','固定电压','前馈','原地转向','前馈+航向','前馈+航向+速度PID','距离直行','比赛自动','连续圆角'][latest[27]] ?? '旧版'} T=${latest[28]}rpm` : '尚无速度采样；请确认遥控器 LOG:ON（Y 切换）';
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
    for (const fd of [csv, raw, motors, headings, velocities, profiles, quality, turns, corners, tuning, routes, pushes, turnProfiles, poses, rolling]) fs.closeSync(fd);
    console.log(`采集结束：连接=${connected}，字节=${bytes}，有效样本=${samples}，电机诊断=${parser.diagnostics}，航向=${parser.headings}，速度闭环=${parser.velocities}，规划=${parser.profiles}，质量=${parser.qualities}，转向=${parser.turns}，圆角=${parser.corners}，移动转向=${parser.rolling}，高速汇总=${parser.tuning}，路线=${parser.routes}，位姿=${parser.poses}，坏行=${parser.malformed}，其他行=${parser.other}，末尾未完成字符=${parser.pending.length}`);
  }
  if (!samples && !parser.poses && !parser.pushes && !parser.routes && !parser.tuning && !parser.corners && !parser.rolling && !parser.turns && !parser.profiles) throw new Error(bytes ? '收到输出，但没有完整运动数据；请确认运行的是包含底盘采样的 Bartmoss' : '没有收到样本；请确认 Bartmoss 运行中、LOG:ON 和无线连接正常');
  if (!samples && latestVelocity) console.log('精简回传：速度由 velocity.csv 的目标减误差获得；基础速度和电机诊断 CSV 仅有表头。');
  else if (!parser.diagnostics) console.log('未收到 LM/LF 角度、电流记录；仅有速度数据，不能完成电机交叉诊断。');
  return { samples, bytes, diagnostics: parser.diagnostics, tuning: parser.tuning, corners: parser.corners, rolling: parser.rolling, turns: parser.turns, turnProfiles: parser.turnProfiles, profiles: parser.profiles, routes: parser.routes, pushes: parser.pushes, poses: parser.poses };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  const seconds = Number(process.argv[2] ?? 30);
  if (!Number.isInteger(seconds) || seconds < 1 || seconds > 600) {
    console.error('用法：node scripts/monitor-drive-websocket.mjs [采集秒数 1–600]');
    process.exitCode = 1;
  } else capture(seconds).catch(error => { console.error(error.message); process.exitCode = 1; });
}
