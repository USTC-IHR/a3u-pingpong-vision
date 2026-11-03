import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
import {STLLoader} from '/vendor/STLLoader.js';

const $ = id => document.getElementById(id);
const canvas = $('photo'), ctx = canvas.getContext('2d');
let frozen = false, points = [], frameId = null, original = null, lastState = null, displayedResult = null;
const scene = new THREE.Scene(); scene.background = new THREE.Color('#1a2633');
const camera = new THREE.PerspectiveCamera(45, 1, .01, 100); camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setPixelRatio(Math.min(devicePixelRatio,2));
$('scene').appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement); controls.target.set(-.5, 0, .2);
scene.add(new THREE.HemisphereLight(0xffffff, 0x555555, 2));
const light = new THREE.DirectionalLight(0xffffff, 2);light.position.set(0,-4,6);scene.add(light);
const table = new THREE.Group();scene.add(table);
function box(size, pos, color, parent=table){const o=new THREE.Mesh(new THREE.BoxGeometry(...size),new THREE.MeshStandardMaterial({color}));o.position.set(...pos);parent.add(o);return o;}
function buildTable(d){
  table.clear();const l=d.length_m,w=d.width_m,h=d.height_m,n=d.net_height_m;
  box([l,w,.03],[0,0,-.015],0x103f96);
  for(const x of [-l/2+.01,l/2-.01])box([.02,w,.003],[x,0,.002],0xf5f5f5);
  for(const y of [-w/2+.01,w/2-.01])box([l,.02,.003],[0,y,.002],0xf5f5f5);
  box([l,.003,.003],[0,0,.002],0xffffff);
  for(let i=0;i<7;i++)box([.004,w+.12,.002],[0,0,i*n/6],0xaab8c8);
  for(let i=0;i<41;i++)box([.004,.002,n],[0,-w/2+i*w/40,n/2],0xaab8c8);
  box([.012,w+.12,.012],[0,0,n],0xffffff);
  for(const x of [-l*.35,l*.35])for(const y of [-w*.35,w*.35])box([.055,.055,h-.03],[x,y,-(h+.03)/2],0x777f86);
  const floor=new THREE.GridHelper(10,20,0x608090,0x35495c);floor.rotation.x=Math.PI/2;floor.position.z=-h;table.add(floor);
}
scene.add(new THREE.AxesHelper(.6));
const robot = new THREE.Group();robot.visible=false;scene.add(robot);
const optical = new THREE.AxesHelper(.25);optical.matrixAutoUpdate=false;optical.visible=false;scene.add(optical);
const links = new Map();let modelErrors=0,modelLoaded=0,modelTotal=0;
const loader = new STLLoader();
function setMatrix(object, rows){object.matrixAutoUpdate=false;object.matrix.set(...rows.flat());object.matrixWorldNeedsUpdate=true;}
function assetStatus(){ $('modelStatus').textContent=`本地网格 ${modelLoaded}/${modelTotal} 已载入${modelErrors?`，${modelErrors} 个失败`:''} · 橙色 = 部分关节参考姿态`; }
async function loadModel(){
 const model=await (await fetch('/api/model')).json();
 modelTotal=model.links.reduce((n,l)=>n+l.visuals.length,0);
 for(const link of model.links){
  const group=new THREE.Group();robot.add(group);links.set(link.name,group);
  for(const v of link.visuals){
   loader.load(v.url,geometry=>{
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0xe6a252,roughness:.7}));
    mesh.position.set(...v.xyz);mesh.rotation.set(v.rpy[0],v.rpy[1],v.rpy[2],'ZYX');mesh.scale.set(...v.scale);group.add(mesh);modelLoaded++;assetStatus();
   },undefined,()=>{modelErrors++;assetStatus();});
  }
 }
 assetStatus();
 if(displayedResult)showResult(displayedResult);
}
loadModel().catch(e=>$('modelStatus').textContent='机器人资产载入失败: '+e.message);
function view(kind){camera.position.set(...({top:[-.8,0,7],side:[-1,-6,1.3],iso:[-4,-4,3]}[kind]));controls.update();}
for(const id of ['top','side','iso'])$(id).onclick=()=>view(id);view('iso');
new ResizeObserver(()=>{const r=$('scene').getBoundingClientRect();renderer.setSize(r.width,r.height);camera.aspect=r.width/r.height;camera.updateProjectionMatrix();}).observe($('scene'));
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
function showResult(r){
 displayedResult=r;robot.visible=!!r?.link_matrices;optical.visible=!!r;
 if(!r){$('measure').textContent='尚无定位结果';$('details').textContent='';return;}
 if(r.link_matrices)for(const [name,m]of Object.entries(r.link_matrices)){if(links.has(name))setMatrix(links.get(name),m);}
 setMatrix(optical,r.T_table_optical);
 const d=r.distances, f=n=>n.toFixed(3)+' m';
 $('measure').textContent=(r.synthetic?'[合成演示] ':'')+(r.method==='manual_snapshot'?'[冻结单帧] ':'')+(r.valid?'':'[有歧义/已过期] ')+(r.T_table_pelvis&&!r.robot_pose_valid?'[机器人位置为近似/参考] ':'')+
 `相机光心 → 台心三维距离：${f(r.camera_to_table_center_3d_m)}`+
 (d?`\n骨盆 → 台面矩形水平最短距离：${f(d.pelvis_to_table_rectangle_xy_m)}\n骨盆 → 台心水平距离：${f(d.pelvis_to_table_center_xy_m)}\n近端有符号 x 间隙：${f(d.near_end_signed_gap_x_m)}\n骨盆在球台系 [x,y,z]：${d.pelvis_table_xyz_m.map(x=>x.toFixed(3)).join(', ')} m`:'\n骨盆位置不可用：需五关节 FK 数据');
 $('measure').style.whiteSpace='pre-line';$('measure').className=r.valid&&r.robot_pose_valid?'good':'bad';
 $('details').textContent=`重投影 RMS: ${r.quality.reprojection_rms_px.toFixed(3)} px\n平面候选误差: ${r.quality.candidate_errors_px.map(x=>x.toFixed(3)).join(', ')} px\n关节: ${r.joint_status}\n${r.warnings.join('\n')}`;
}
function draw(){
 if(!original)return;canvas.width=original.naturalWidth;canvas.height=original.naturalHeight;ctx.drawImage(original,0,0);
 const outline=displayedResult?.table_edges_px||displayedResult?.table_outline_px;
 if(outline){ctx.beginPath();outline.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();ctx.strokeStyle='#44ffaa';ctx.lineWidth=3;ctx.stroke();}
 points.forEach((p,i)=>{ctx.beginPath();ctx.arc(...p,7,0,2*Math.PI);ctx.fillStyle='#ffb548';ctx.fill();ctx.font='bold 25px sans-serif';ctx.fillText(String(i+1),p[0]+10,p[1]-10);});
 $('solve').disabled=points.length!==4||lastState?.preview;
}
async function loadFrame(id){
 const img=new Image();img.src='/api/frame/'+id;await img.decode();if(frozen)return;original=img;frameId=id;draw();
}
function freeze(){if(!original||frozen)return;frozen=true;post('/api/pin',{frame_id:frameId}).catch(e=>$('message').textContent=e.message);$('message').textContent='已冻结帧 '+frameId+'。依次点选四角；结果仅对应这张图。';}
$('freeze').onclick=freeze;
$('resume').onclick=()=>{frozen=false;points=[];displayedResult=null;$('message').textContent='';};
$('clear').onclick=()=>{points=[];displayedResult=null;showResult(null);draw();};
canvas.onclick=e=>{freeze();if(points.length>=4)return;const r=canvas.getBoundingClientRect();points.push([(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height]);draw();};
async function post(path,body){const res=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await res.json();if(!res.ok)throw Error(data.error);return data;}
$('solve').onclick=async()=>{try{const r=await post('/api/solve',{frame_id:frameId,points});showResult(r);draw();$('message').textContent='已解算冻结帧，原图和结果已保存。恢复实时后不沿用该帧位姿。';}catch(e){showResult(null);$('message').textContent=e.message;}};
$('save').onclick=async()=>{try{const r=await post('/api/save',{frame_id:frameId});$('message').textContent='已保存 '+r.saved;}catch(e){$('message').textContent=e.message;}};
let built=false;
async function poll(){
 try{
  const s=await(await fetch('/api/state')).json();lastState=s;
  $('state').textContent=`${s.source==='demo'?'合成演示，非实机 | ':''}${s.camera_id} | ${s.source} | ${s.status} | 帧 ${s.frame_id??'-'} | 接收年龄 ${s.age_s?.toFixed(2)??'-'}s${frozen?' | 冻结单帧':''}${s.preview?' | 未标定预览，仅可保存图像':''}\n${s.error||s.detection||''}`;
  $('state').className=s.status==='live'&&!s.error?'good':'bad';
  if(!built){buildTable(s.table);built=true;}
  if(!frozen&&s.frame_id){
   if(frameId!==s.frame_id)await loadFrame(s.frame_id);
   if(!frozen){showResult(s.result);draw();}
  }
 }catch(e){$('state').textContent='连接中断: '+e.message; if(!frozen){showResult(null);draw();}}
 setTimeout(poll,200);
}
poll();
