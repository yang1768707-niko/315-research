// Real driver/store sources, stateful NOR simulation; no hardware access.
const fs=require('fs'),path=require('path'),cp=require('child_process'),assert=require('assert/strict');
const root=path.resolve(__dirname,'..'),out=path.join(root,'Debug/qspi-store-tests');
fs.mkdirSync(out,{recursive:true});
const cc=process.env.NATIVE_CC||'gcc',prefix=JSON.parse(process.env.NATIVE_CC_ARGS||'[]');
function run(command,args){
  const r=cp.spawnSync(command,args,{cwd:root,stdio:'inherit'});
  if(r.error)throw r.error;
  if(r.status!==0)throw new Error(`${command} failed: status=${r.status} signal=${r.signal}`);
}
function definition(text,name){
  const match=new RegExp('(?:static\\s+)?[\\w*]+\\s+'+name+'\\([^;]*?\\)\\s*\\{','g').exec(text);
  assert(match,name);let depth=1,i=match.index+match[0].length;
  for(;depth&&i<text.length;i++){if(text[i]==='{')depth++;if(text[i]==='}')depth--;}
  assert.equal(depth,0);return text.slice(match.index,i);
}
const entry=fs.readFileSync(path.join(root,'Core/Src/app_entry.c'),'utf8');
const app=fs.readFileSync(path.join(root,'STM32_WPAN/App/custom_app.c'),'utf8');
fs.writeFileSync(path.join(out,'stop2-under-test.inc'),
  definition(app,'Custom_APP_PrepareStop2')+'\n'+
  ['UTIL_SEQ_PreIdle','UTIL_SEQ_Idle','UTIL_SEQ_PostIdle'].map(n=>definition(entry,n)).join('\n'));
for(const profile of [0,1]){
  const exe=path.join(out,`test_qspi_store_${profile}.exe`);
  run(cc,[...prefix,'-std=c11','-Wall','-Wextra','-Werror','-DQSPI_NOR_PROGRAM_TIMING='+profile,
    '-I'+path.join(root,'tests/fakes'),'-I'+path.join(root,'Core/Inc'),
    path.join(root,'tests/test_qspi_store.c'),path.join(root,'Core/Src/qspi_nor.c'),
    path.join(root,'Core/Src/zigbee_vibration_store.c'),'-o',exe]);run(exe,[]);
}
const exe=path.join(out,'test_stop2_guard.exe');
run(cc,[...prefix,'-std=c11','-Wall','-Wextra','-Werror','-I'+out,
  path.join(root,'tests/test_stop2_guard.c'),'-o',exe]);run(exe,[]);
