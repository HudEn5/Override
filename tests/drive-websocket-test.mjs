import assert from 'node:assert/strict';
import fs from 'node:fs';
import { DriveLines, HEADER, MOTOR_HEADER, HEADING_HEADER, VELOCITY_HEADER, PROFILE_HEADER, QUALITY_HEADER, TURN_HEADER, TURN_PROFILE_HEADER, CORNER_HEADER, ROLLING_HEADER, TUNING_HEADER, ROUTE_HEADER, PUSH_HEADER } from '../scripts/monitor-drive-websocket.mjs';
import { TeachPoints } from '../scripts/capture-teach.mjs';

const teach = new TeachPoints();
assert.equal(teach.push('#web#DWP,1,1,3,-2.0,4.0,'), null);
assert.equal(teach.push('90.0\r\nDWP,1,3,3,0.0,0.0,0.0\n'), null);
assert.equal(teach.push('DWP,2,1,2,1.0,2.0,3.0\nDWP,2,2,2,0,xx,0\n'), null);
assert.deepEqual(teach.push('DWP,2,2,2,-3.5,4.0,85.0\n'), [[1, 1, 2, 3], [2, -3.5, 4, 85]]);

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
const qualityRows = [];
const qualityParser = new DriveLines(noProfile, noProfile, noProfile, noProfile, noProfile,
  row => qualityRows.push(row));
const quality = ['10', '4000', '0', '8', '1', '1', '0.00', '0.00', '1', '1'];
const qualityStream = `#web#DSQ,${quality.join(',')}\nDSQ,broken\n`;
for (let i = 0; i < qualityStream.length; i += 3) qualityParser.push(qualityStream.slice(i, i + 3));
assert.equal(QUALITY_HEADER.split(',').length, 10);
assert.deepEqual(qualityRows, [quality]);
assert.equal(qualityParser.qualities, 1);
assert.equal(qualityParser.malformed, 1);
parser.push('x'.repeat(65537));
assert.equal(parser.pending, '');
// 已有真实旧版日志也必须完整兼容，不改动原文件。
for (const [name, count] of [['1794412458993', 68], ['1794413751103', 119]]) {
  const url = ['../build/data/history/', '../build/data/', '../build/']
    .map(directory => new URL(`${directory}drive-websocket-${name}.log`, import.meta.url))
    .find(candidate => fs.existsSync(candidate));
  if (!url) continue;
  let n = 0;
  const old = new DriveLines(row => { n++; assert.equal(row.length, 29); assert.equal(row[27], '-1'); });
  old.push(fs.readFileSync(url, 'utf8'));
  assert.equal(n, count);
  assert.equal(old.malformed, 0);
}
const turnRows = [];
const turnParser = new DriveLines(noProfile, noProfile, noProfile, noProfile, noProfile, noProfile,
  row => turnRows.push(row));
const turn = ['11', '4020', '1000', '45', '90', '45', '30', '20', '-900', '920', '1'];
const turnStream = `#web#DTN,${turn.join(',')}\nDTN,broken\nDTN,${[...turn.slice(0, 10), 'NaN'].join(',')}\n`;
for (let i = 0; i < turnStream.length; i += 3) turnParser.push(turnStream.slice(i, i + 3));
assert.equal(TURN_HEADER.split(',').length, 11);
assert.deepEqual(turnRows, [turn]);
assert.equal(turnParser.turns, 1);
assert.equal(turnParser.malformed, 2);
const turnProfileRows=[];
const turnProfileParser=new DriveLines(noProfile,noProfile,noProfile,noProfile,noProfile,noProfile,
  noProfile,noProfile,noProfile,noProfile,noProfile,row=>turnProfileRows.push(row));
const turnProfile=['11','4020','200','368','180','592','314','1','3'];
const turnProfileStream=`#web#DTA,${turnProfile.join(',')}\nDTA,broken\n`;
for(let i=0;i<turnProfileStream.length;i+=3)turnProfileParser.push(turnProfileStream.slice(i,i+3));
assert.equal(TURN_PROFILE_HEADER.split(',').length,9);
assert.deepEqual(turnProfileRows,[turnProfile]);assert.equal(turnProfileParser.turnProfiles,1);
for(const [index,value] of [[2,'-1'],[5,'-10'],[6,'NaN'],[7,'5'],[7,'1.5'],[8,'Infinity']]) {
  const bad=[...turnProfile];bad[index]=value;turnProfileParser.push(`DTA,${bad.join(',')}\n`);
}
assert.equal(turnProfileParser.malformed,7);
console.log('Angular profile protocol checks passed: DTA fragments, phase and invalid reference fields.');
const cornerRows = [];
const cornerParser = new DriveLines(noProfile, noProfile, noProfile, noProfile, noProfile, noProfile, noProfile,
  row => cornerRows.push(row));
const corner = ['11','4020','2000','2','1','600','900','10','11','1','25','50','30','70','1200','1800'];
const cornerStream = `#web#DCR,${corner.join(',')}\nDCR,broken\nDCR,${[...corner.slice(0,15),'NaN'].join(',')}\n`;
for (let i = 0; i < cornerStream.length; i += 3) cornerParser.push(cornerStream.slice(i, i + 3));
assert.equal(CORNER_HEADER.split(',').length, 16);
assert.deepEqual(cornerRows, [corner]);
assert.equal(cornerParser.corners, 1);
assert.equal(cornerParser.malformed, 2);
console.log('WebSocket protocol checks passed: DS2, DLM, DHD, DVL, DTP, DSQ, DTN, DCR, legacy DSV, fragments and invalid rows.');

const tuningRows = [];
const tuningParser = new DriveLines(()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{},()=>{}, row=>tuningRows.push(row));
const tuning = ['11','4000','1','2','-1','2','3500','-1497','-3','268','1.2','15','0','600','1'];
const tuningStream = `#web#DHS,${tuning.join(',')}\nDHS,broken\nDHS,${[...tuning.slice(0,14),'NaN'].join(',')}\n`;
for(let i=0;i<tuningStream.length;i+=3)tuningParser.push(tuningStream.slice(i,i+3));
assert.equal(TUNING_HEADER.split(',').length,15);
assert.deepEqual(tuningRows,[tuning]);assert.equal(tuningParser.tuning,1);assert.equal(tuningParser.malformed,2);
console.log('High-speed summary protocol checks passed.');

// 实测出现过列数与有限数值均正确、标志却被错位成小数的坏行。
const rejected = new DriveLines(() => assert.fail('语义破损的速度行应拒绝'));
for (const [index, value] of [[3, '16'], [23, '0.00'], [24, '2'], [25, '12001'], [27, '9']]) {
  const bad = [...current]; bad[index] = value;
  rejected.push(`DS2,${bad.join(',')}\n`);
}
assert.equal(rejected.malformed, 5);
for (const [index, value] of [[3, '5'], [4, '0'], [5, '9'], [9, '-1'], [14, '1.00'], [13, '299'], [5, '1']]) {
  const bad = [...tuning]; bad[index] = value;
  tuningParser.push(`DHS,${bad.join(',')}\n`);
}
assert.equal(tuningParser.tuning, 1);
assert.equal(tuningParser.malformed, 9);
// 精简高速流允许没有 DS2；目标减误差仍可还原两侧反馈速度。
let compactVelocity, compactTuning;
const compact = new DriveLines(() => assert.fail('精简流没有 DS2'), () => {}, () => {},
  row => { compactVelocity = row; }, () => {}, () => {}, () => {}, () => {},
  row => { compactTuning = row; });
const compactStream = `DTP,${profile.join(',')}\nDHD,${heading.join(',')}\nDVL,${velocity.join(',')}\nDHS,${tuning.join(',')}\n`;
for (let i = 0; i < compactStream.length; i += 7) compact.push(compactStream.slice(i, i + 7));
assert.equal(compact.malformed, 0);
assert.equal(compact.tuning, 1);
assert.deepEqual(compactTuning, tuning);
assert.equal(Number(compactVelocity[2]) - Number(compactVelocity[4]), 90);
assert.equal(Number(compactVelocity[3]) - Number(compactVelocity[5]), 110);
console.log('Semantic rejection and compact high-speed stream checks passed.');
const compactCornerRows = [];
const compactCorner = new DriveLines(noProfile, noProfile, noProfile, () => {}, noProfile, noProfile, noProfile,
  row => compactCornerRows.push(row));
const cornerOnlyStream = `DCR,${corner.join(',')}\nDVL,${velocity.join(',')}\n`;
for (let i = 0; i < cornerOnlyStream.length; i += 7) compactCorner.push(cornerOnlyStream.slice(i, i + 7));
assert.equal(compactCorner.corners, 1); assert.equal(compactCorner.velocities, 1);
assert.equal(compactCorner.malformed, 0); assert.deepEqual(compactCornerRows, [corner]);
console.log('Compact corner stream checks passed without DS2 or DHS.');
const routeRows = [];
const routeParser = new DriveLines(noProfile, noProfile, noProfile, noProfile, noProfile, noProfile, noProfile, noProfile, noProfile,
  row => routeRows.push(row));
const routeRow = ['12','5000','1','2','0','1','0','1','1'];
const routeDone = ['13','5500','1','3','3','0','1','0','2'];
const routeStream = `#web#DRA,${routeRow.join(',')}\nDRA,${routeDone.join(',')}\n`;
for (let i = 0; i < routeStream.length; i += 3) routeParser.push(routeStream.slice(i, i + 3));
assert.equal(ROUTE_HEADER.split(',').length, 9); assert.deepEqual(routeRows, [routeRow, routeDone]);
for (const [index, value] of [[3, '-1'], [4, '6'], [6, '1'], [7, '1.0'], [8, '5']]) {
  const bad = [...routeRow]; bad[index] = value; routeParser.push(`DRA,${bad.join(',')}\n`);
}
assert.equal(routeParser.routes, 2); assert.equal(routeParser.malformed, 5);
console.log('Route status protocol checks passed: fragmented WAIT/DONE and invalid state rejection.');
const pushRows = [];
const pushParser = new DriveLines(noProfile,noProfile,noProfile,noProfile,noProfile,noProfile,noProfile,noProfile,noProfile,()=>{}, row=>pushRows.push(row));
const push = ['10','5000','800','1000','4000','4000','1','0','0','0'];
const settle = ['11','5200','1000','1000','0','0','1','1','0','0'];
const pushStream = `DRA,12,5400,1,0,4,1,0,0,1\nDPS,${push.join(',')}\nDPS,${settle.join(',')}\n`;
for (let i=0;i<pushStream.length;i+=3) pushParser.push(pushStream.slice(i,i+3));
assert.equal(PUSH_HEADER.split(',').length,10); assert.deepEqual(pushRows,[push,settle]);
assert.equal(pushParser.routes,1); assert.equal(pushParser.pushes,2); assert.equal(pushParser.malformed,0);
for (const [index,value] of [[3,'5001'],[4,'12001'],[5,'-1'],[6,'5'],[7,'1']]) {
  const bad=[...push];bad[index]=value;pushParser.push(`DPS,${bad.join(',')}\n`);
}
assert.equal(pushParser.malformed,5); assert.equal(pushParser.pushes,2);
const reversePush = [...push];
reversePush[4] = reversePush[5] = '-4000';
pushParser.push(`DPS,${reversePush.join(',')}\n`);
assert.deepEqual(pushRows[2], reversePush);
console.log('Push protocol checks passed: stalled force/settle fragments, route motion=4 and invalid duration/voltage/state rejection.');

const poseRows = [];
const poseParser = new DriveLines(...Array(12).fill(() => {}), row => poseRows.push(row));
const poseRecords = [
  ['0','0','0','0','0','0','0'],
  ['1','500','-123.45','678.90','361.25','1','0'],
  ['2','500','-123.45','678.90','361.25','0','1'],
];
const poseStream = poseRecords.map(row => `DPO,${row.join(',')}\n`).join('');
for (let i = 0; i < poseStream.length; i += 3) poseParser.push(poseStream.slice(i, i + 3));
assert.deepEqual(poseRows, poseRecords);
for (const [index, value] of [[0,'-1'],[1,'-1'],[2,'NaN'],[5,'2'],[6,'1']]) {
  const bad = [...poseRecords[1]]; bad[index] = value;
  poseParser.push(`DPO,${bad.join(',')}\n`);
}
poseParser.push('DPO,3,600,1,2,3,1\n');
assert.equal(poseParser.poses, 3); assert.equal(poseParser.malformed, 6);
console.log('Pose protocol passed: fragmented WAIT/OK/RESET, signed coordinates, invalid flags and truncated rows.');

// 新移动转向与旧圆角分别记录；旧DCR的字段数/余量含义保留。
assert.equal(ROLLING_HEADER.split(',').length, 18);
const rollingRows = [];
const rollingParser = new DriveLines(...Array(13).fill(() => {}), row => rollingRows.push(row));
const rolling = [...corner, '2', '4'];
const rollingStream = `DRL,${rolling.join(',')}\nDRA,12,5400,1,0,5,1,0,0,1\n`;
for (let i = 0; i < rollingStream.length; i += 3) rollingParser.push(rollingStream.slice(i, i + 3));
assert.deepEqual(rollingRows, [rolling]);
assert.equal(rollingParser.rolling, 1);
assert.equal(rollingParser.corners, 0);
assert.equal(rollingParser.routes, 1);
assert.equal(rollingParser.malformed, 0);
for (const [index, value] of [[3,'5'],[4,'5'],[14,'12001'],[16,'5'],[17,'0'],[17,'1'],[6,'NaN']]) {
  const bad = [...rolling]; bad[index] = value;
  rollingParser.push(`DRL,${bad.join(',')}\n`);
}
rollingParser.push(`DRL,${corner.join(',')}\n`);
assert.equal(rollingParser.malformed, 8);
console.log('Rolling protocol checks passed: separate DRL fragments, motion=5 and invalid field rejection.');

const rollingConfigError = [...corner, '0', '0'];
rollingConfigError[4] = '4';
rollingParser.push(`DRL,${rollingConfigError.join(',')}\n`);
assert.equal(rollingParser.rolling, 2);
assert.equal(rollingParser.malformed, 8);

const sixteenTurns = [...rolling.slice(0, 16), '16', '16'];
rollingParser.push(`DRL,${sixteenTurns.join(',')}\n`);
assert.equal(rollingParser.rolling, 3);
assert.equal(rollingParser.malformed, 8);
const tooManyTurns = [...rolling.slice(0, 16), '0', '17'];
rollingParser.push(`DRL,${tooManyTurns.join(',')}\n`);
assert.equal(rollingParser.malformed, 9);
