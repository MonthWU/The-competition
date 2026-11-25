// Purpose: check HMI protocol meanings against wire vectors, including the reported black-block frame.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {readFileSync}=require('node:fs');
const {join}=require('node:path');
const vm=require('node:vm');
const html=readFileSync(join(__dirname,'../src/car26_system/car26/hmi/index.html'),'utf8');
const source=html.split('/* SERIAL_DECODER_START */')[1]?.split('/* SERIAL_DECODER_END */')[0];
assert.ok(source,'decoder block must exist');
const context=vm.createContext({TextDecoder});
vm.runInContext(source,context);

/* 函数名字：decode；函数作用：运行页面实际解码器；参数意义与范围：hex：待测HEX；direction：tx或rx，默认tx。 */
function decode(hex,direction='tx'){return context.decodeSerialFrame(hex,direction)}
/* 函数名字：batch；函数作用：构造测试用物块报文；参数意义与范围：records：(类别字节,dx,dy)数组，偏差为int16。 */
function batch(records){const b=Buffer.alloc(4+5*records.length);b[0]=0x5b;b[1]=0x71;b[2]=records.length;b[b.length-1]=0x5d;
for(let i=0;i<records.length;i++){const [code,dx,dy]=records[i],offset=3+i*5;b[offset]=code;b.writeInt16LE(dx,offset+1);b.writeInt16LE(dy,offset+3)}
return b.toString('hex')}

/* 函数名字：reportedFrame；函数作用：复现用户反馈并核对黑色类别和小端偏差；参数意义与范围：无。 */
function reportedFrame(){const text=decode('5B 71 01 42 B8 00 28 00 5D');assert.match(text,/种类数：1/);assert.match(text,/黑色物块（0x42）/);assert.match(text,/dx=\+184 px，dy=\+40 px/);assert.doesNotMatch(text,/\\u000|\[q/)}
test('user vector decodes to one black block at +184,+40',reportedFrame);

/* 函数名字：allClasses；函数作用：核对六种语义码，禁止采用模型输出顺序；参数意义与范围：无。 */
function allClasses(){const text=decode(batch([[0x41,1,2],[0x42,3,4],[0x43,5,6],[0x44,7,8],[0x45,9,10],[0x46,11,12]]));
assert.match(text,/种类数：6/);for(const name of ['红色','黑色','绿色','黄色','浅蓝色','深蓝色'])assert.ok(text.includes(name+'物块'))}
test('six class codes follow UART semantics',allClasses);

/* 函数名字：signedOffsets；函数作用：核对补码边界及载荷含括号/逗号；参数意义与范围：无。 */
function signedOffsets(){const text=decode(batch([[0x41,-32768,32767],[0x42,0x5d,0x5b],[0x43,-1,0x2c]]));assert.match(text,/dx=-32768 px，dy=\+32767 px/);assert.match(text,/dx=\+93 px，dy=\+91 px/);assert.match(text,/dx=-1 px，dy=\+44 px/)}
test('signed offsets and embedded delimiter bytes',signedOffsets);

/* 函数名字：emptyFrame；函数作用：零种物块不虚构坐标；参数意义与范围：无。 */
function emptyFrame(){const text=decode('5b71005d');assert.match(text,/种类数：0/);assert.ok(!text.includes('dx='))}
test('zero-class packet has no fabricated block',emptyFrame);

/* 函数名字：invalidBatch；函数作用：拒绝数量长度不符、未知类别和重复类别；参数意义与范围：无。 */
function invalidBatch(){for(const hex of ['5b71015d',batch([[0x47,0,0]]),batch([[0x41,0,0],[0x41,1,1]])])assert.match(decode(hex),/^未解码/)}
test('malformed class packets are not guessed',invalidBatch);

/* 函数名字：states；函数作用：核对三状态与0x61下位机确认方向语义；参数意义与范围：无。 */
function states(){for(const [code,name] of [['60','二维码识别'],['61','物料识别'],['62','标靶识别']]){
assert.ok(decode('5b'+code+'2c015d').includes(name));assert.match(decode('5b'+code+'2c005d'),/退出/)}
assert.match(decode('5b612c015d','rx'),/下位机确认.*请求圆盘中心检测/);assert.match(decode('5b612c025d'),/^未解码/)}
test('state notifications and MCU confirmation',states);

/* 函数名字：disc；函数作用：九字节圆盘回复独立解析，不能误当五字节状态；参数意义与范围：无。 */
function disc(){assert.match(decode('5b612cf4ff2c08005d'),/圆盘中心偏差.*\n.*dx=-12 px，dy=\+8 px/);assert.match(decode('5b612c5b5d2c2c005d'),/dx=\+23899 px，dy=\+44 px/)}
test('nine-byte disc reply uses signed LE16 fields',disc);

/* 函数名字：pose；函数作用：核对角度单位、来源及偏差；参数意义与范围：无。 */
function pose(){assert.match(decode('5b72002efbc0feb4005d'),/YOLO粗定位\nα=-12.34°，dx=-320 px，dy=\+180 px/);assert.match(decode('5b72017d003b0076ff5d'),/圆心精定位\nα=1.25°，dx=\+59 px，dy=-138 px/);assert.match(decode('5b72027d003b0076ff5d'),/^未解码/)}
test('target angle scale and coordinate source',pose);

/* 函数名字：qr；函数作用：只有0x70内容按UTF-8解码，保留合法原文且拒绝非法字节；参数意义与范围：无。 */
function qr(){const value=' 原文[中文 ';const hex=Buffer.concat([Buffer.from([0x5b,0x70]),Buffer.from(value),Buffer.from([0x5d])]).toString('hex');assert.equal(decode(hex),'二维码结果（0x70）\n'+value);assert.match(decode('5b70ff5d'),/^未解码/);assert.match(decode('5b705d'),/^未解码/)}
test('only QR payload uses UTF-8',qr);

/* 函数名字：asciiFrames；函数作用：保留定义的ASCII握手/控制及校验正确的地图；参数意义与范围：无。 */
function asciiFrames(){assert.match(decode(Buffer.from('[ack]').toString('hex')),/握手确认/);assert.match(decode(Buffer.from('[align_done]').toString('hex'),'rx'),/完成标靶精定位/);assert.match(decode(Buffer.from('[1 13 0C]').toString('hex')),/障碍格数：1.*格号：13/);assert.match(decode(Buffer.from('[1 13 00]').toString('hex')),/^未解码/)}
test('ASCII commands and map checksum',asciiFrames);

/* 函数名字：controlCommands；函数作用：验证网页对应串口命令的中文解释，避免q字符误判为0x71物块包；参数意义与范围：无。 */
function controlCommands(){for(const [command,label] of [['qr_reset','重置二维码结果'],['task_switch','切换二维码/物块任务'],['task_qr','选择二维码任务'],['task_object','选择物块任务']])assert.equal(decode(Buffer.from('['+command+']').toString('hex'),'rx'),'控制命令 · '+label)}
test('web controls share MCU command meanings',controlCommands);

/* 函数名字：controlOutcomes；函数作用：执行页面实际的按ID查询逻辑，验证后续MCU状态不丢失网页确认；参数意义与范围：无。 */
function controlOutcomes(){const source=html.split('/* CONTROL_RESULT_START */')[1].split('/* CONTROL_RESULT_END */')[0];vm.runInContext(source,context);assert.equal(context.controlResult({request_id:'uart',status:'done',results:[{request_id:'web',status:'done'}]},'web').status,'done');assert.equal(context.controlResult({request_id:'web',status:'pending'},'web').status,'pending');assert.equal(context.controlResult(undefined,'web'),null);assert.equal(context.controlResult({request_id:'uart'},'missing'),null)}
test('MCU activity cannot overwrite the awaited web outcome',controlOutcomes);

/* 函数名字：fragments；函数作用：所有截断前缀、粘包及非法HEX不能伪装成完整包；参数意义与范围：无。 */
function fragments(){const hex=batch([[0x42,93,40]]);for(let i=0;i<hex.length;i+=2)assert.match(decode(hex.slice(0,i)),/^未解码/);for(const bad of [hex+hex,'5b71005','zz','5b99005d','qabc'])assert.match(decode(bad),/^未解码/)}
test('fragments and joined packets remain raw',fragments);
