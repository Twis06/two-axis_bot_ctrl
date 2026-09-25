'use strict';
const data=JSON.parse(document.getElementById('data').textContent);
const finite=x=>typeof x==='number'&&Number.isFinite(x);
const median=a=>{a=a.filter(finite).sort((x,y)=>x-y);return a.length?(a[(a.length-1)>>1]+a[a.length>>1])/2:null};
const fmt=(x,n=3)=>finite(x)?x.toFixed(n):'unavailable';
const esc=x=>String(x).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const palette=['#849699','#285e66','#7b678c','#008079','#ae7522','#a15846'];
function chart(target,groups,unit){
 const max=Math.max(.1,...groups.flatMap(g=>g.values).filter(finite))*1.12, W=820,left=185,right=85,top=45,row=49,H=top+groups.length*row+42;
 let s=`<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="${esc(unit)} by controller. Bars represent medians; circles represent individual runs."><text x="${left}" y="20" font-size="13">${esc(unit)} · median bars and individual runs</text>`;
 for(let i=0;i<=4;i++){const x=left+(W-left-right)*i/4;s+=`<line x1="${x}" y1="30" x2="${x}" y2="${H-30}" stroke="#dce3e4"/><text x="${x}" y="${H-10}" text-anchor="middle" font-size="12">${fmt(max*i/4,2)}</text>`}
 groups.forEach((g,i)=>{const y=top+i*row,m=median(g.values),scale=(W-left-right)/max;s+=`<text x="${left-12}" y="${y+16}" text-anchor="end" font-size="13">${esc(g.name)}</text>`;if(finite(m))s+=`<rect x="${left}" y="${y}" width="${m*scale}" height="23" fill="${palette[i%palette.length]}" opacity=".72"/><text x="${W-5}" y="${y+17}" text-anchor="end" font-size="12">${fmt(m)}</text>`;g.values.forEach((v,j)=>{if(finite(v))s+=`<circle cx="${left+v*scale}" cy="${y+11+(j%5-2)*3}" r="3" fill="${palette[i%palette.length]}" stroke="white" stroke-width=".7"><title>${esc(g.name)} · ${esc(g.labels[j])}: ${fmt(v,5)} ${esc(unit)}</title></circle>`})});
 document.getElementById(target).innerHTML=s+'</svg>';
}
const loads=[...new Set(data.learning.filter(r=>r.split==='heldout').map(r=>JSON.stringify(r.load)))];
for(const l of loads){const op=document.createElement('option');op.value=l;op.textContent=JSON.parse(l).map(v=>(v>=0?'+':'')+v.toFixed(2)).join(', ');document.getElementById('load').append(op)}
function learning(){
 const l=document.getElementById('load').value,limit=document.getElementById('limit').value;
 const rows=data.learning.filter(r=>r.split==='heldout'&&(l==='all'||JSON.stringify(r.load)===l)&&(limit==='all'||r.limit===Number(limit)));
 const names={'int0.5':'Integral ×0.5','int1':'Integral ×1 (baseline)','int2':'Integral ×2 (ineligible)','adaptive':'Adaptive','oracle':'Oracle + governor','oracle_ff':'Oracle FF only'};
 const groups=Object.keys(names).map(v=>{const rs=rows.filter(r=>r.variant===v);return{name:names[v],values:rs.map(r=>r.primary_deg),labels:rs.map(r=>`seed ${r.seed}, load ${r.load}, ${r.limit} A`),rows:rs}});
 chart('learning-chart',groups,'Primary RMS (°)');
 const key=r=>JSON.stringify([r.load,r.limit,r.seed]);const base=new Map(rows.filter(r=>r.variant==='int1').map(r=>[key(r),r]));
 const a=rows.filter(r=>r.variant==='adaptive');const reductions=a.map(r=>{const b=base.get(key(r));return finite(r.primary_deg)&&finite(b.primary_deg)&&b.primary_deg>0?100*(1-r.primary_deg/b.primary_deg):null});
 document.getElementById('learning-summary').textContent=`Selected adaptive cases: ${a.length}. Median paired reduction: ${fmt(median(reductions),1)}%. Usable estimate in ${a.filter(r=>r.learn_usable_max>0).length}/${a.length}; completed ${a.filter(r=>r.completed).length}/${a.length}. Filtered subsets do not replace the full registered gate.`;
 document.getElementById('learning-table').innerHTML='<div class="table-wrap" tabindex="0" role="region" aria-label="Selected learning results"><table><thead><tr><th>Variant</th><th>Primary median (°)</th><th>Finite metric / runs</th><th>Completed</th><th>WD trips</th></tr></thead><tbody>'+groups.map(g=>`<tr><td>${esc(g.name)}</td><td>${fmt(median(g.values))}</td><td>${g.values.filter(finite).length}/${g.rows.length}</td><td>${g.rows.filter(r=>r.completed).length}/${g.rows.length}</td><td>${g.rows.reduce((s,r)=>s+r.wd_trips,0)}</td></tr>`).join('')+'</tbody></table></div>';
}
function prediction(){const sel=document.getElementById('prediction-metric'),metric=sel.value;chart('prediction-chart',[[1,true],[.9,true],[1,false],[.9,false]].map(([k,ff])=>{const rs=data.prediction.filter(r=>r.motor_strength_ratio===k&&r.use_yaw_ff===ff&&r.yaw_info==='estimate');return{name:`Kt ×${k.toFixed(2)} · FF ${ff?'on':'off'}`,values:rs.map(r=>r[metric]),labels:rs.map(r=>'seed '+r.seed)}}),sel.options[sel.selectedIndex].textContent)}
['load','limit'].forEach(id=>document.getElementById(id).addEventListener('change',learning));document.getElementById('prediction-metric').addEventListener('change',prediction);learning();prediction();
document.getElementById('expand').addEventListener('click',e=>{const ds=[...document.querySelectorAll('details.evidence')],open=ds.some(d=>!d.open);ds.forEach(d=>d.open=open);e.currentTarget.textContent=open?'Collapse all evidence':'Expand all evidence'});
document.getElementById('print').addEventListener('click',()=>window.print());
const dialog=document.getElementById('figure-dialog');document.querySelectorAll('.figure-button').forEach(b=>b.addEventListener('click',()=>{const im=b.querySelector('img');dialog.querySelector('img').src=im.src;dialog.querySelector('img').alt=im.alt;dialog.querySelector('p').textContent=b.closest('figure').querySelector('figcaption').textContent;dialog.showModal()}));dialog.addEventListener('click',e=>{if(e.target===dialog)dialog.close()});
const nav=[...document.querySelectorAll('nav a')];const observer=new IntersectionObserver(entries=>{for(const e of entries)if(e.isIntersecting)nav.forEach(a=>a.classList.toggle('active',a.hash==='#'+e.target.id))},{rootMargin:'0px 0px -65% 0px'});document.querySelectorAll('main>section>h2[id]').forEach(h=>observer.observe(h));
