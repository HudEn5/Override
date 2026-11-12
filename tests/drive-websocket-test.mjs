import assert from 'node:assert/strict';
import fs from 'node:fs';
import { DriveLines, HEADER, MOTOR_HEADER, HEADING_HEADER, VELOCITY_HEADER, PROFILE_HEADER } from '../scripts/monitor-drive-websocket.mjs';

// 协议契约测试：模拟串口跨消息分片，不能将半行误写成完整样本。
const received = [];
const parser = new DriveLines(row => received.push(row));
const legacy = Array(27).fill('0');
const current = [...legacy, '2', '-150.0'];
current[25] = '-2968'; current[26] = '-2886';
const stream = `BMV,1\r\nDSV,${legacy.join(',')}\r\n#web#DS2,${current.join(',')}\nDS2,broken\nDSV,${current.join(',')}\n`;
for (let i = 0; i < stream.length; i += 7) parser.push(stream.slice(i, i + 7));
assert.equal(HEADER.split(',').length, 29);
assert.deepEqual(received, [[...legacy, '-1', '0'], current]);
assert.equal(parser.malformed, 2);
assert.equal(parser.other, 1);
assert.equal(parser.pending, '');
parser.push(`DS2,${[...current.slice(0, 28), 'NaN'].join(',')}\n`);
assert.equal(parser.malformed, 3);
const diagnosticRows = [];
const diagnosticParser = new DriveLines(() => assert.fail('诊断行不能写进速度 CSV'), row => diagnosticRows.push(row));
const diagnostic = ['10', '200.0', '-123.45', '1.234', '-120.00', '1.100', '3'];
const diagnosticStream = `DLM,${diagnostic.join(',')}\nDLM,broken\n`;
for (let i = 0; i < diagnosticStream.length; i += 3) diagnosticParser.push(diagnosticStream.slice(i, i + 3));
assert.equal(MOTOR_HEADER.split(',').length, 7);
assert.deepEqual(diagnosticRows, [diagnostic]);
assert.equal(diagnosticParser.diagnostics, 1);
assert.equal(diagnosticParser.malformed, 1);
const headingRows = [];
const headingParser = new DriveLines(() => assert.fail('航向不能写入速度 CSV'),
  () => assert.fail('航向不能写入电机 CSV'), row => headingRows.push(row));
const heading = ['10', '210.0', '12.0', '10.0', '-2.0', '-160', '1', '1', '1'];
const headingStream = `#web#DHD,${heading.join(',')}\nDHD,broken\nDHD,${[...heading.slice(0, 8), 'NaN'].join(',')}\n`;
for (let i = 0; i < headingStream.length; i += 5) headingParser.push(headingStream.slice(i, i + 5));
assert.equal(HEADING_HEADER.split(',').length, 9);
assert.deepEqual(headingRows, [heading]);
assert.equal(headingParser.headings, 1);
assert.equal(headingParser.malformed, 2);
const velocityRows = [];
const velocityParser = new DriveLines(() => assert.fail('闭环诊断不能写入速度 CSV'),
  () => assert.fail('闭环诊断不能写入电机 CSV'),
  () => assert.fail('闭环诊断不能写入航向 CSV'), row => velocityRows.push(row));
const velocity = ['10', '200.0', '100', '100', '10', '-10', '60', '-60', '1'];
const velocityStream = `#web#DVL,${velocity.join(',')}\nDVL,broken\n`;
for (let i = 0; i < velocityStream.length; i += 3) velocityParser.push(velocityStream.slice(i, i + 3));
assert.equal(VELOCITY_HEADER.split(',').length, 9);
assert.deepEqual(velocityRows, [velocity]);
assert.equal(velocityParser.velocities, 1);
assert.equal(velocityParser.malformed, 1);
const profileRows = [];
const noProfile = () => assert.fail('梯形数据不能写入其他 CSV');
const profileParser = new DriveLines(noProfile, noProfile, noProfile, noProfile,
  row => profileRows.push(row));
const profile = ['10', '4000', '1000', '200', '100', '195', '101', '305', '2500', '1', '2'];
const profileStream = `#web#DTP,${profile.join(',')}\nDTP,broken\nDTP,${[...profile.slice(0, 10), 'NaN'].join(',')}\n`;
for (let i = 0; i < profileStream.length; i += 3) profileParser.push(profileStream.slice(i, i + 3));
assert.equal(PROFILE_HEADER.split(',').length, 11);
assert.deepEqual(profileRows, [profile]);
assert.equal(profileParser.profiles, 1);
assert.equal(profileParser.malformed, 2);
parser.push('x'.repeat(65537));
assert.equal(parser.pending, '');
// 已有真实旧版日志也必须完整兼容，不改动原文件。
for (const [name, count] of [['1794412458993', 68], ['1794413751103', 119]]) {
  const url = new URL(`../build/drive-websocket-${name}.log`, import.meta.url);
  if (!fs.existsSync(url)) continue;
  let n = 0;
  const old = new DriveLines(row => { n++; assert.equal(row.length, 29); assert.equal(row[27], '-1'); });
  old.push(fs.readFileSync(url, 'utf8'));
  assert.equal(n, count);
  assert.equal(old.malformed, 0);
}
console.log('WebSocket protocol checks passed: DS2, DLM, DHD, DVL, DTP, legacy DSV, fragments and invalid rows.');
