// Host tests extract production mode functions; no MCU or debugger access.
const fs=require('fs'),path=require('path'),assert=require('assert/strict'),cp=require('child_process');
const root=path.resolve(__dirname,'..');
const source=fs.readFileSync(path.join(root,'STM32_WPAN/App/custom_app.c'),'utf8');
const bleSource=fs.readFileSync(path.join(root,'STM32_WPAN/App/app_ble.c'),'utf8');
const out=path.join(root,'Debug/ble-mode-tests');fs.mkdirSync(out,{recursive:true});
const pd=source.match(/\/\* USER CODE BEGIN PD \*\/([\s\S]*?)\/\* USER CODE END PD \*\//);assert(pd);
function definition(name, content=source) {
 const re=new RegExp('(?:static\\s+)?[\\w*]+\\s+'+name+'\\([^;]*?\\)\\s*\\{','g');
 const m=re.exec(content);assert(m,'Missing production definition '+name);let depth=1,i=re.lastIndex;
 for(;i<content.length&&depth;i++){if(content[i]==='{')depth++;if(content[i]==='}')depth--;}
 assert.equal(depth,0);return content.slice(m.index,i);
}
const names=['DT_ScheduleWorkMode','DT_ConfirmModeRestart','DT_LoadWorkMode','DT_SaveWorkMode','DT_QueueWorkModeStatus','DT_ProcessWorkMode','DT_ServiceWorkModeTask','DT_CommWorkMode','DT_HandleSensorCtrl','Custom_APP_GetWorkMode','Custom_APP_GetConfiguredWorkMode','DT_CommBleSend'];
fs.writeFileSync(path.join(out,'mode-under-test.inc'),pd[1]+'\n'+names.map(name=>definition(name)).join('\n'));
fs.writeFileSync(path.join(out,'bond-under-test.inc'),definition('DT_BLE_BondSecurityTask',bleSource));
const exe=path.join(out,'test_ble_work_mode.exe');
function run(command,args) {
 const result=cp.spawnSync(command,args,{stdio:'inherit',cwd:root});
 if(result.error)throw result.error;if(result.status!==0)process.exit(result.status||1);
}
// Prefer an explicitly selected compiler, then PATH GCC, then installed MATLAB LCC64.
let cc=process.env.NATIVE_CC;
if(!cc&&cp.spawnSync('gcc',['--version']).status===0)cc='gcc';
const originalGcc='F:/WorkAPP/QT/Tools/mingw1310_64/bin/gcc.exe';
if(!cc&&fs.existsSync(originalGcc))cc=originalGcc;
if(cc) {
 run(cc,['-std=c11','-Wall','-Wextra','-Werror','-I'+out,'-I'+path.join(root,'Core/Inc'),
   path.join(__dirname,'test_ble_work_mode.c'),path.join(root,'Core/Src/comm_protocol.c'),'-o',exe]);
} else {
 const lcc=process.env.LCC_ROOT||'D:/Program Files/MATLAB/R2024b/sys/lcc64/lcc64';
 const compiler=path.join(lcc,'bin/lcc64.exe'),linker=path.join(lcc,'bin/lcclnk64.exe');
 assert(fs.existsSync(compiler),'Set NATIVE_CC to host GCC or LCC_ROOT to installed LCC64');
 const objects=[];
 for(const unit of ['tests/test_ble_work_mode.c','Core/Src/comm_protocol.c']) {
  const obj=path.join(out,path.basename(unit,'.c')+'.obj');objects.push(obj);
  run(compiler,['-I'+path.join(lcc,'include64'),'-I'+out,'-I'+path.join(root,'Core/Inc'),'-Fo'+obj,path.join(root,unit)]);
 }
 run(linker,['-L'+path.join(lcc,'lib64'),'-o',exe,...objects]);
}
run(exe,[]);
