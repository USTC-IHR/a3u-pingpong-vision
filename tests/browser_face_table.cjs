// Optional local UI smoke test. No production ROS topics or external sites.
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || '/home/mine/.openclaw/tools/node-v24.22.1/lib/node_modules/openclaw/node_modules/playwright-core');
const fs=require('fs'), assert=require('assert');
(async()=>{
 const browser=await chromium.launch({executablePath:process.env.CHROME_BIN||'/usr/bin/google-chrome',headless:true,args:['--no-sandbox','--enable-unsafe-swiftshader']});
 const errors=[];
 try{
  const page=await browser.newPage({viewport:{width:1600,height:1100}});
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(process.env.FACE_TABLE_URL||'http://127.0.0.1:18092');
  await page.waitForFunction(()=>document.querySelector('#measure').textContent.includes('骨盆在球台系'));
  await page.waitForFunction(()=>{
   const s=document.querySelector('#modelStatus').textContent.match(/(\d+)\/(\d+) 已载入/);
   return s&&s[1]===s[2];
  },null,{timeout:45000});
  const assets=await page.locator('#modelStatus').innerText();
  assert(!assets.includes('失败'),assets);
  await page.locator('#freeze').click();
  const state=await page.evaluate(async()=>await(await fetch('/api/state')).json());
  for(const point of state.result.table_outline_px){
   await page.locator('#photo').evaluate((el,p)=>{
    const r=el.getBoundingClientRect();el.dispatchEvent(new MouseEvent('click',{bubbles:true,clientX:r.left+p[0]*r.width/el.width,clientY:r.top+p[1]*r.height/el.height}));
   },point);
  }
  assert(await page.locator('#solve').isEnabled());
  await page.locator('#solve').click();
  await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('已解算冻结帧'));
  const measurement=await page.locator('#measure').innerText();
  assert(measurement.includes('[冻结单帧]'));
  await page.locator('#top').click();await page.locator('#side').click();await page.locator('#iso').click();
  fs.mkdirSync('logs/face-table-validation',{recursive:true});
  await page.screenshot({path:'logs/face-table-validation/browser-desktop.png',fullPage:true});
  await page.locator('#save').click();
  await page.waitForFunction(()=>document.querySelector('#message').textContent.includes('已保存'));
  await page.locator('#resume').click();
  await page.waitForFunction(()=>!document.querySelector('#measure').textContent.includes('[冻结单帧]'));
  await page.setViewportSize({width:390,height:844});
  assert(!(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth)),'mobile horizontal overflow');
  await page.screenshot({path:'logs/face-table-validation/browser-mobile.png',fullPage:true});
  assert.equal(errors.length,0,errors.join(';'));
  const result={result:'PASS',assets,measurement,console_errors:errors,checks:['ArUco live result','all STL assets','four-corner UI solve','frozen snapshot label','camera views','save image','resume live','mobile layout']};
  fs.writeFileSync('logs/face-table-validation/browser-result.json',JSON.stringify(result,null,2));
  console.log(JSON.stringify(result));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exit(1)});
