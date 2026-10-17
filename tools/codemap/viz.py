# ================================================================================================
#  tools/codemap/viz.py -- build/codemap/architecture.html: the map as a picture to point at.
#
#  One self-contained page (no network, no library): the source directories as boxes, callers above
#  callees (layers by longest path over the directory graph, its cycles collapsed first); a click
#  opens a directory's files the same way, and a file's functions; a function shows its callers and
#  callees side by side. A scene picker colours everything by what that scene reaches (blue) and
#  runs every frame (orange), grey where it reaches nothing; the panel lists the switches the scene
#  decides. NOTES: anything selected can carry a note ("app should not call hal directly"); the
#  notes stay in this browser and EXPORT as text to paste into a conversation -- the way to say
#  "this is not how I see it" about a precise box.
# ================================================================================================
import base64
import json
import os


def bits(ids, index):
    b = bytearray((len(index) + 7) // 8)
    for i in ids:
        k = index.get(i)
        if k is not None:
            b[k >> 3] |= 1 << (k & 7)
    return base64.b64encode(bytes(b)).decode()


def data(g, sc):
    import graph as G
    fns = sorted((i for i, s in g.sym.items() if s["repo"] and s["file"] and (s["kind"] in G.FN_KINDS or s["lang"] == "hlsl")
                  and (s["file"].startswith("src/") or s["file"].startswith("shaders/"))),
                 key=lambda i: (g.sym[i]["file"], g.sym[i]["line"], i))
    index = {i: k for k, i in enumerate(fns)}
    files = sorted({g.sym[i]["file"] for i in fns})
    fidx = {f: k for k, f in enumerate(files)}
    dirs = sorted({f.rsplit("/", 1)[0] for f in files})
    didx = {d: k for k, d in enumerate(dirs)}
    lines = {}
    for f in files:
        try:
            with open(os.path.join(G.ROOT, f), encoding="utf-8", errors="replace") as fh:
                lines[f] = sum(1 for _ in fh)
        except OSError:
            lines[f] = 0
    kinds = {"Constructor": "C", "Destructor": "D", "HlslFunction": "H"}
    out_fns = []
    for i in fns:
        s = g.sym[i]
        e = g.entries.get(i)
        out_fns.append([g.short(i), fidx[s["file"]], s["line"], s["end"], kinds.get(s["kind"], "F"),
                        f"{e['profile']} {e['entry']}" if e else ""])
    edges = {}
    for lst in g.out.values():
        for e in lst:
            a, b = index.get(e.src), index.get(e.dst)
            if a is None or b is None or a == b:
                continue
            k = (a, b)
            cur = edges.get(k)
            kind = {"shader": 1, "hlsl": 2}.get(e.kind, 0)
            if cur is None:
                edges[k] = [a, b, kind, 1 if e.guard else 0, 1 if e.loop else 0, 1]
            else:
                cur[3] &= 1 if e.guard else 0       # unguarded if any site is
                cur[4] |= 1 if e.loop else 0
                cur[5] += 1
    ovr = sorted({(index[b], index[o]) for b, os_ in g.overrides.items() for o in os_ if b in index and o in index})
    scenes = []
    for s in sc.ok:
        p = s["path"]
        mp = os.path.join(G.ROOT, "build", "codemap", "scenes", p[len("scenes/"):].rsplit(".", 1)[0] + ".map")
        sw = []
        if os.path.exists(mp):
            with open(mp, encoding="utf-8") as fh:
                for ln in fh:
                    if ln.startswith("- ") and "==> false" in ln:
                        sw.append(ln[2:].strip())
                    if len(sw) >= 25:
                        break
        scenes.append({"path": p, "reach": bits(s["reach"], index), "frame": bits(s["frame"], index), "switches": sw})
    return {
        "dirs": dirs, "files": [[f, didx[f.rsplit("/", 1)[0]], lines[f]] for f in files], "fns": out_fns,
        "edges": sorted(edges.values()), "ovr": ovr, "scenes": scenes,
        "any": bits(sc.any_reach, index), "anyFrame": bits(sc.any_frame, index),
    }


def write(g, sc, path):
    blob = json.dumps(data(g, sc), separators=(",", ":"))
    html = PAGE.replace("/*DATA*/null", blob)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(html)


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GAGAME architecture</title>
<style>
:root{--bg:#f7f7f5;--panel:#fff;--ink:#1d1d1b;--mute:#6b6b66;--line:#d9d8d2;--reach:#2f6fdd;--frame:#e07b1f;--off:#c9c8c2;--sel:#111;--edge:#8a8a84}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#171716;--panel:#20201e;--ink:#ecebe6;--mute:#9a9992;--line:#3a3936;--reach:#6d9cf0;--frame:#f0a050;--off:#4a4945;--sel:#fff;--edge:#7d7c76}}
:root[data-theme=dark]{--bg:#171716;--panel:#20201e;--ink:#ecebe6;--mute:#9a9992;--line:#3a3936;--reach:#6d9cf0;--frame:#f0a050;--off:#4a4945;--sel:#fff;--edge:#7d7c76}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.4 system-ui,Segoe UI,sans-serif}
header{display:flex;flex-wrap:wrap;gap:8px;align-items:center;padding:8px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
header h1{font-size:15px;margin:0 12px 0 0}select,input,button{font:inherit;color:var(--ink);background:var(--bg);border:1px solid var(--line);border-radius:4px;padding:3px 6px}
button{cursor:pointer}#crumb{color:var(--mute)}#crumb a{color:var(--reach);cursor:pointer;text-decoration:none}
main{display:grid;grid-template-columns:1fr 360px;height:calc(100vh - 46px)}@media(max-width:800px){main{grid-template-columns:1fr;height:auto}}
#view{overflow:auto;position:relative}svg{display:block}#side{border-left:1px solid var(--line);background:var(--panel);overflow:auto;padding:10px 14px}
.box rect{stroke:var(--line);stroke-width:1;rx:4}.box text{fill:var(--ink);font-size:11px;pointer-events:none}.box{cursor:pointer}.box.sel rect{stroke:var(--sel);stroke-width:2}
.e{fill:none;stroke:var(--edge);opacity:.45}.e.up{stroke:var(--frame);stroke-dasharray:4 3;opacity:.7}.e.hot{stroke:var(--reach);opacity:.9}.lbl{fill:var(--mute);font-size:10px}
h2{font-size:14px;margin:6px 0}h3{font-size:12px;margin:12px 0 4px;color:var(--mute);text-transform:uppercase;letter-spacing:.04em}
.lst div{padding:1px 0;cursor:pointer;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.lst div:hover{text-decoration:underline}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px;vertical-align:middle}code{font:11px ui-monospace,Consolas,monospace;word-break:break-all}
textarea{width:100%;min-height:60px;font:inherit;color:var(--ink);background:var(--bg);border:1px solid var(--line);border-radius:4px}
.legend span{margin-right:10px;color:var(--mute)}.sw{font:11px ui-monospace,Consolas,monospace;margin:2px 0;color:var(--mute)}
</style></head><body>
<header><h1>GAGAME architecture</h1>
<label>scene <select id="scene"></select></label>
<input id="find" placeholder="find a function or file" size="24">
<span class="legend"><span><i class="dot" style="background:var(--reach)"></i>reached</span><span><i class="dot" style="background:var(--frame)"></i>per frame</span><span><i class="dot" style="background:var(--off)"></i>not reached</span></span>
<span id="crumb"></span>
<button id="exp" title="copy every note as text">export notes</button></header>
<main><div id="view"><svg id="svg"></svg></div><aside id="side"></aside></main>
<script>
const D=/*DATA*/null;
const NS="http://www.w3.org/2000/svg";
const $=s=>document.querySelector(s);
const dec=b=>{const s=atob(b),u=new Uint8Array(s.length);for(let i=0;i<s.length;i++)u[i]=s.charCodeAt(i);return u};
const has=(u,k)=>(u[k>>3]>>(k&7))&1;
const F=D.fns.map((f,k)=>({k,n:f[0],file:f[1],l:f[2],e:f[3],kind:f[4],entry:f[5]}));
const fileFns=D.files.map(()=>[]);F.forEach(f=>fileFns[f.file].push(f.k));
const dirFiles=D.dirs.map(()=>[]);D.files.forEach((f,k)=>dirFiles[f[1]].push(k));
const out=F.map(()=>[]),inc=F.map(()=>[]);D.edges.forEach(e=>{out[e[0]].push(e);inc[e[1]].push(e)});
const ovr=F.map(()=>[]),base=F.map(()=>[]);D.ovr.forEach(([b,o])=>{ovr[b].push(o);base[o].push(b)});
const bases=k=>{const out=[],st=[...base[k]];while(st.length){const b=st.pop();out.push(b);st.push(...base[b])}return out};
let S={reach:dec(D.any),frame:dec(D.anyFrame),switches:[],path:""};
const sel=$("#scene");sel.innerHTML='<option value="-1">any scene</option>'+D.scenes.map((s,k)=>`<option value="${k}">${s.path}</option>`).join("");
sel.onchange=()=>{const k=+sel.value;S=k<0?{reach:dec(D.any),frame:dec(D.anyFrame),switches:[],path:""}:{reach:dec(D.scenes[k].reach),frame:dec(D.scenes[k].frame),switches:D.scenes[k].switches,path:D.scenes[k].path};draw()};
let notes={};try{notes=JSON.parse(localStorage.getItem("codemap.notes")||"{}")}catch(e){}
const saveNotes=()=>{try{localStorage.setItem("codemap.notes",JSON.stringify(notes))}catch(e){}};
let view={level:"top",dir:-1,file:-1,fn:-1},UP=[];
function frac(ks){let r=0,f=0;ks.forEach(k=>{r+=has(S.reach,k);f+=has(S.frame,k)});return [ks.length?r/ks.length:0,ks.length?f/ks.length:0]}
function el(t,a,p){const e=document.createElementNS(NS,t);for(const k in a)e.setAttribute(k,a[k]);if(p)p.appendChild(e);return e}
// Layered layout. The directories call each other in cycles, so the order comes from a weighted
// feedback arc set (Eades, Lin, Smyth 1993, greedy): sinks to the bottom, sources to the top, else
// the node with the most call weight out over in; the calls left pointing up are the fewest the
// cycles allow, drawn dashed (they are what a layering would forbid). Layers: longest path over
// the downward calls; order in a layer: barycenter sweeps.
function layout(n,E){const outw=Array(n).fill(0),inw=Array(n).fill(0),adj=[...Array(n)].map(()=>new Map()),radj=[...Array(n)].map(()=>new Map());
 E.forEach(([a,b,w])=>{if(a===b)return;adj[a].set(b,(adj[a].get(b)||0)+w);radj[b].set(a,(radj[b].get(a)||0)+w);outw[a]+=w;inw[b]+=w});
 const alive=new Set([...Array(n).keys()]),s1=[],s2=[];
 const drop=v=>{alive.delete(v);adj[v].forEach((w,b)=>{inw[b]-=w});radj[v].forEach((w,a)=>{outw[a]-=w})};
 while(alive.size){let moved=true;while(moved){moved=false;for(const v of [...alive]){if(outw[v]<=0){s2.unshift(v);drop(v);moved=true}else if(inw[v]<=0){s1.push(v);drop(v);moved=true}}}
  if(!alive.size)break;let best=-1,bd=-Infinity;for(const v of alive){const d=outw[v]-inw[v];if(d>bd||(d===bd&&v<best)){bd=d;best=v}}s1.push(best);drop(best)}
 const order=s1.concat(s2),rank=Array(n);order.forEach((v,i)=>rank[v]=i);
 const layer=Array(n).fill(0);order.forEach(v=>{E.forEach(([a,b])=>{if(a===v&&rank[b]>rank[a])layer[b]=Math.max(layer[b],layer[a]+1)})});
 const L=Math.max(0,...layer)+1,rows=[...Array(L)].map(()=>[]);order.forEach(v=>rows[layer[v]].push(v));
 const pos=Array(n).fill(0);rows.forEach(r=>r.forEach((v,i)=>pos[v]=i));const nb=[...Array(n)].map(()=>[]);E.forEach(([a,b])=>{nb[a].push(b);nb[b].push(a)});
 for(let it=0;it<4;it++)rows.forEach(r=>{r.sort((a,b)=>{const m=v=>nb[v].length?nb[v].reduce((s,x)=>s+pos[x],0)/nb[v].length:pos[v];return m(a)-m(b)||a-b});r.forEach((v,i)=>pos[v]=i)});
 return {layer,rows,rank}}
function boxes(items,E,title){const svg=$("#svg");svg.innerHTML="";const W=Math.max(640,$("#view").clientWidth-20);
 const {layer,rows}=layout(items.length,E);UP=E.filter(([a,b])=>a!==b&&layer[b]<=layer[a]).map(([a,b,w])=>[items[a].name,items[b].name,w]).sort((x,y)=>y[2]-x[2]);const bw=Math.min(190,Math.max(110,(W-40)/Math.max(...rows.map(r=>r.length))-14)),bh=44,gy=70;
 const P=items.map(()=>({x:0,y:0}));rows.forEach((r,li)=>{const tot=r.length*(bw+14);r.forEach((v,i)=>{P[v]={x:(W-tot)/2+i*(bw+14)+7,y:20+li*(bh+gy)}})});
 svg.setAttribute("width",W);svg.setAttribute("height",40+rows.length*(bh+gy));
 const mx=Math.max(1,...E.map(e=>e[2]));const g1=el("g",{},svg);
 E.forEach(([a,b,w])=>{if(a===b)return;const A=P[a],B=P[b];const up=layer[b]<=layer[a];const x1=A.x+bw/2,y1=up?A.y:A.y+bh,x2=B.x+bw/2,y2=up?B.y+bh:B.y;
  const p=el("path",{class:"e"+(up?" up":""),d:`M${x1},${y1} C${x1},${(y1+y2)/2} ${x2},${(y1+y2)/2} ${x2},${y2}`,"stroke-width":1+3*Math.log(1+w)/Math.log(1+mx)},g1);
  p.dataset.a=a;p.dataset.b=b;el("title",{},p).textContent=`${items[a].name} -> ${items[b].name}: ${w} calls${up?" (UPWARD: against the layering)":""}`});
 items.forEach((it,v)=>{const g=el("g",{class:"box"+(it.sel?" sel":"")},svg);const [r,f]=frac(it.fns);
  el("rect",{x:P[v].x,y:P[v].y,width:bw,height:bh,fill:r?"var(--reach)":"var(--off)","fill-opacity":r?0.15+0.55*r:0.5},g);
  if(f)el("rect",{x:P[v].x,y:P[v].y+bh-5,width:bw*f,height:5,fill:"var(--frame)"},g);
  el("text",{x:P[v].x+6,y:P[v].y+16},g).textContent=it.name.length>28?"..."+it.name.slice(-27):it.name;
  el("text",{x:P[v].x+6,y:P[v].y+31,class:"lbl"},g).textContent=`${it.fns.length} fns · ${Math.round(r*100)}% reached`+(notes[it.key]?" · note":"");
  el("title",{},g).textContent=it.name;
  g.onmouseenter=()=>g1.querySelectorAll("path").forEach(p=>p.classList.toggle("hot",+p.dataset.a===v||+p.dataset.b===v));
  g.onclick=()=>it.open()});
 $("#crumb").innerHTML=title}
function edgesBetween(groupOf,n){const m=new Map();D.edges.forEach(e=>{const a=groupOf(e[0]),b=groupOf(e[1]);if(a<0||b<0||a===b)return;const k=a*n+b;m.set(k,(m.get(k)||0)+e[5])});
 return [...m].map(([k,w])=>[Math.floor(k/n),k%n,w])}
function draw(){if(view.level==="top")overview();else if(view.level==="dir")openDir(view.dir);else if(view.level==="file")openFile(view.file,view.fn);}
function overview(){view={level:"top",dir:-1,file:-1,fn:-1};const items=D.dirs.map((d,k)=>({name:d,key:"dir:"+d,fns:dirFiles[k].flatMap(f=>fileFns[f]),open:()=>{view={level:"dir",dir:k};draw()}}));
 const dOf=f=>D.files[F[f].file][1];boxes(items,edgesBetween(dOf,D.dirs.length),"<b>all subsystems</b>");panelTop()}
function openDir(k){const fl=dirFiles[k];const loc=new Map(fl.map((f,i)=>[f,i]));
 const items=fl.map(f=>({name:D.files[f][0].split("/").pop(),key:"file:"+D.files[f][0],fns:fileFns[f],open:()=>{view={level:"file",dir:k,file:f,fn:-1};draw()}}));
 const ofs=f=>{const i=loc.get(F[f].file);return i===undefined?-1:i};boxes(items,edgesBetween(ofs,fl.length),`<a onclick="overview()">all</a> / <b>${D.dirs[k]}</b>`);panelDir(k)}
function openFile(f,fn){view={level:"file",dir:D.files[f][1],file:f,fn};const svg=$("#svg");svg.innerHTML="";
 $("#crumb").innerHTML=`<a onclick="overview()">all</a> / <a onclick="openDir(${D.files[f][1]})">${D.dirs[D.files[f][1]]}</a> / <b>${D.files[f][0].split("/").pop()}</b>`;
 const ks=fileFns[f];if(fn<0)fn=ks[0];if(fn===undefined){panelFile(f);return}
 const W=Math.max(640,$("#view").clientWidth-20);const cal=[...new Set(inc[fn].map(e=>e[0]).concat(bases(fn).flatMap(b=>inc[b].map(e=>e[0]))))],cle=[...new Set(out[fn].map(e=>e[1]).concat(ovr[fn]))];
 const rowH=22,n=Math.max(cal.length,cle.length,1);svg.setAttribute("width",W);svg.setAttribute("height",Math.max(300,60+Math.min(n,60)*rowH));
 const cx=W/2-90,cy=30;const col=k=>has(S.frame,k)?"var(--frame)":has(S.reach,k)?"var(--reach)":"var(--off)";
 function node(k,x,y,w,center){const g=el("g",{class:"box"+(center?" sel":"")},svg);el("rect",{x,y,width:w,height:18,fill:col(k),"fill-opacity":center?.35:.2},g);
  el("text",{x:x+5,y:y+13},g).textContent=(F[k].n.length>34?"..."+F[k].n.slice(-33):F[k].n);el("title",{},g).textContent=`${F[k].n}  ${D.files[F[k].file][0]}:${F[k].l}`;
  g.onclick=()=>openFile(F[k].file,k);return g}
 node(fn,cx,cy+Math.min(n,60)*rowH/2-9,180,true);
 const ln=(x1,y1,x2,y2)=>el("path",{class:"e",d:`M${x1},${y1} C${(x1+x2)/2},${y1} ${(x1+x2)/2},${y2} ${x2},${y2}`},svg);
 cal.slice(0,60).forEach((k,i)=>{node(k,20,cy+i*rowH,200);ln(220,cy+i*rowH+9,cx,cy+Math.min(n,60)*rowH/2)});
 cle.slice(0,60).forEach((k,i)=>{node(k,W-220,cy+i*rowH,200);ln(cx+180,cy+Math.min(n,60)*rowH/2,W-220,cy+i*rowH+9)});
 panelFn(f,fn,cal,cle)}
function noteBox(key,label){const v=notes[key]||"";return `<h3>note on ${label}</h3><textarea data-key="${key}" placeholder="what is wrong here, what you want instead">${v.replace(/</g,"&lt;")}</textarea>`}
function wire(){document.querySelectorAll("textarea[data-key]").forEach(t=>t.oninput=()=>{const k=t.dataset.key;if(t.value.trim())notes[k]=t.value;else delete notes[k];saveNotes()});
 document.querySelectorAll("[data-fn]").forEach(d=>d.onclick=()=>{const k=+d.dataset.fn;openFile(F[k].file,k)})}
function lst(ks){return `<div class="lst">${ks.slice(0,80).map(k=>`<div data-fn="${k}"><i class="dot" style="background:${has(S.frame,k)?"var(--frame)":has(S.reach,k)?"var(--reach)":"var(--off)"}"></i>${F[k].n} <span style="color:var(--mute)">${D.files[F[k].file][0].split("/").pop()}:${F[k].l}</span></div>`).join("")}${ks.length>80?`<div>+${ks.length-80} more</div>`:""}</div>`}
function panelTop(){const n=F.length,[r,f]=frac(F.map(x=>x.k));$("#side").innerHTML=`<h2>${S.path||"any scene"}</h2><div>${n} functions · ${Math.round(r*n)} reached · ${Math.round(f*n)} per frame</div>
 <h3>switches this scene decides</h3>${S.switches.length?S.switches.map(s=>`<div class="sw">${s.replace(/</g,"&lt;")}</div>`).join(""):'<div class="sw">pick a scene above</div>'}
 <h3>upward calls (against the layering)</h3>${UP.length?UP.slice(0,12).map(u=>`<div class="sw">${u[0]} -> ${u[1]}: ${u[2]}</div>`).join(""):'<div class="sw">none</div>'}
 <h3>how to read</h3><div>Boxes are source directories; a caller sits above what it calls. Dashed orange: calls that point up, against the order the call weight implies (what a strict layering would forbid). Fill: share of its functions the scene reaches; orange bar: share run every frame. Click to open.</div>${noteBox("all","the whole")}`;wire()}
function panelDir(k){const ks=dirFiles[k].flatMap(f=>fileFns[f]);const [r,f]=frac(ks);$("#side").innerHTML=`<h2>${D.dirs[k]}</h2><div>${dirFiles[k].length} files · ${ks.length} functions · ${Math.round(r*100)}% reached · ${Math.round(f*100)}% per frame</div>${noteBox("dir:"+D.dirs[k],D.dirs[k])}
 <h3>not reached in this scene</h3>${lst(ks.filter(x=>!has(S.reach,x)))}`;wire()}
function panelFile(f){$("#side").innerHTML=`<h2>${D.files[f][0]}</h2><div>${D.files[f][2]} lines · ${fileFns[f].length} functions</div>${noteBox("file:"+D.files[f][0],"this file")}<h3>functions</h3>${lst(fileFns[f])}`;wire()}
function panelFn(f,fn,cal,cle){const x=F[fn];const q=`py -3 tools/codemap/q.py fn ${x.n}`+(S.path?` --scene ${S.path}`:"");
 $("#side").innerHTML=`<h2>${x.n}</h2><div><code>${D.files[f][0]}:${x.l}-${x.e}</code>${x.entry?`<div>shader entry ${x.entry}</div>`:""}</div>
 <div>${has(S.frame,fn)?"runs every frame":has(S.reach,fn)?"reached":"not reached"} in ${S.path||"some scene"}</div><h3>ask</h3><code>${q}</code><br><code>py -3 tools/codemap/q.py why ${x.n} --scene ${S.path||"scenes/..."}</code>
 ${noteBox("fn:"+D.files[f][0]+":"+x.n,x.n)}${bases(fn).length?`<div>overrides ${bases(fn).map(b=>F[b].n).join(", ")}: its callers count as callers here</div>`:""}<h3>callers (${cal.length})</h3>${lst(cal)}<h3>calls (${cle.length})</h3>${lst(cle)}<h3>this file</h3>${lst(fileFns[f])}`;wire()}
$("#find").onchange=e=>{const s=e.target.value.trim().toLowerCase();if(!s)return;const k=F.find(x=>x.n.toLowerCase()===s)||F.find(x=>x.n.toLowerCase().includes(s));
 if(k){openFile(k.file,k.k);return}const fi=D.files.findIndex(x=>x[0].toLowerCase().includes(s));if(fi>=0)openFile(fi,-1)};
$("#exp").onclick=()=>{const t=Object.keys(notes).sort().map(k=>`- ${k}: ${notes[k].trim()}`).join("\n")||"(no notes yet)";
 const txt=`Notes on the GAGAME architecture map${S.path?" (scene "+S.path+")":""}:\n${t}`;(navigator.clipboard?navigator.clipboard.writeText(txt):Promise.reject()).then(()=>alert("notes copied"),()=>prompt("copy these notes",txt))};
window.onresize=()=>draw();overview();
</script></body></html>
"""
