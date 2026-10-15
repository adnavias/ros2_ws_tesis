// Interfaz web del control del brazo.
// Recibe el estado por Server-Sent Events (/api/stream) y manda comandos
// con POST /api/comando. No usa librerías externas (funciona sin internet).
'use strict';

const $ = (id) => document.getElementById(id);
// Quita ceros finales sólo de la parte decimal: 50 -> '50', 6.540 -> '6.54'.
function numTexto(x, dec) {
  const t = Number(x).toFixed(dec);
  return t.includes('.') ? t.replace(/0+$/, '').replace(/\.$/, '') : t;
}
const S = { estado: null, clave: '', pausa: false, hist: {}, sucias: new Set(), vista: 'polos' };
const MAX_PUNTOS = 2400;

// ------------------------------------------------------------ utilidades
function toast(msg, error = false) {
  const t = $('toast');
  t.textContent = msg;
  t.className = 'toast visible' + (error ? ' error' : '');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => { t.className = 'toast'; }, error ? 6000 : 3000);
}
async function comando(accion, datos = {}) {
  try {
    const r = await fetch('/api/comando', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(Object.assign({ accion }, datos)),
    });
    const j = await r.json();
    if (j.mensaje) toast(j.mensaje, !j.ok);
    return j;
  } catch (e) {
    toast('Sin conexión con la interfaz: ' + e, true);
    return { ok: false };
  }
}
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d);
function css(nombre) { return getComputedStyle(document.documentElement).getPropertyValue(nombre).trim(); }
function el(tag, attrs = {}, hijos = []) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v; else if (k === 'text') e.textContent = v;
    else e.setAttribute(k, v);
  }
  for (const h of hijos) e.appendChild(h);
  return e;
}
function numInput(id, valor, paso = 0.01) {
  const i = el('input', { type: 'number', step: String(paso), id });
  if (valor !== null && valor !== undefined) i.value = numTexto(valor, 4);
  i.addEventListener('input', () => S.sucias.add(id));
  return i;
}
function fijarSiLibre(id, valor, dec = 4) {
  const i = $(id);
  if (!i || S.sucias.has(id) || document.activeElement === i) return;
  if (valor === null || valor === undefined) { i.value = ''; return; }
  i.value = numTexto(valor, dec);
}

// ------------------------------------------------------------ gráficas
function dibujar(canvas, series, ventana, extra = {}) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w * dpr) { canvas.width = w * dpr; canvas.height = h * dpr; }
  const c = canvas.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, w, h);
  const izq = 46, der = 8, arr = 8, aba = 18;
  let tmax = -Infinity, ymin = Infinity, ymax = -Infinity;
  for (const s of series) if (s.t.length) tmax = Math.max(tmax, s.t[s.t.length - 1]);
  if (!Number.isFinite(tmax)) return;
  const tmin = tmax - ventana;
  for (const s of series) for (let k = 0; k < s.t.length; k++) {
    if (s.t[k] < tmin || s.y[k] === null) continue;
    ymin = Math.min(ymin, s.y[k]); ymax = Math.max(ymax, s.y[k]);
  }
  if (!Number.isFinite(ymin)) return;
  if (extra.simetrico) { const m = Math.max(Math.abs(ymin), Math.abs(ymax)); ymin = -m; ymax = m; }
  if (ymax - ymin < 1e-6) { ymax += 0.5; ymin -= 0.5; }
  const pad = 0.08 * (ymax - ymin); ymin -= pad; ymax += pad;
  const X = (t) => izq + (t - tmin) / ventana * (w - izq - der);
  const Y = (y) => arr + (ymax - y) / (ymax - ymin) * (h - arr - aba);
  c.strokeStyle = css('--rejilla'); c.lineWidth = 1; c.fillStyle = css('--tenue');
  c.font = '11px system-ui'; c.textAlign = 'right'; c.textBaseline = 'middle';
  for (let k = 0; k <= 4; k++) {
    const y = ymin + k * (ymax - ymin) / 4, py = Y(y);
    c.beginPath(); c.moveTo(izq, py); c.lineTo(w - der, py); c.stroke();
    c.fillText(y.toFixed(Math.abs(ymax - ymin) < 2 ? 2 : 1), izq - 4, py);
  }
  c.textAlign = 'center'; c.textBaseline = 'alphabetic';
  for (let k = 0; k <= 4; k++) {
    const t = tmin + k * ventana / 4;
    c.fillText(`${(t - tmax).toFixed(0)} s`, X(t), h - 4);
  }
  for (const s of series) {
    c.strokeStyle = s.color; c.lineWidth = s.ancho || 1.6;
    c.setLineDash(s.guion ? [5, 4] : []);
    c.beginPath(); let mov = true;
    for (let k = 0; k < s.t.length; k++) {
      if (s.t[k] < tmin) continue;
      if (s.y[k] === null) { mov = true; continue; }
      const px = X(s.t[k]), py = Y(s.y[k]);
      if (mov) { c.moveTo(px, py); mov = false; } else c.lineTo(px, py);
    }
    c.stroke();
  }
  c.setLineDash([]);
}
function historial(nombre) {
  if (!S.hist[nombre]) S.hist[nombre] = { t: [], pos: [], ref: [], err: [], cmd: [], lim: [], limn: [] };
  return S.hist[nombre];
}
function guardarHistorial(e) {
  if (!e.nombres) return;
  e.nombres.forEach((n, k) => {
    const h = historial(n);
    h.t.push(e.t); h.pos.push(e.posicion_deg[k]); h.ref.push(e.referencia_deg[k]);
    h.err.push(e.error_deg[k]); h.cmd.push(e.mando_valor[k]);
    const l = e.limite_mando[k]; h.lim.push(l); h.limn.push(l === null ? null : -l);
    if (h.t.length > MAX_PUNTOS) for (const a of Object.values(h)) a.shift();
  });
}
function redibujar() {
  if (S.pausa || !S.estado) return;
  const nombre = $('sel-graf').value; const h = S.hist[nombre];
  if (!h) return;
  const v = Number($('sel-ventana').value);
  dibujar($('g-pos'), [{ t: h.t, y: h.ref, color: css('--c2'), guion: true },
                       { t: h.t, y: h.pos, color: css('--c1') }], v);
  dibujar($('g-err'), [{ t: h.t, y: h.err, color: css('--c3') }], v, { simetrico: true });
  dibujar($('g-cmd'), [{ t: h.t, y: h.lim, color: css('--c5'), guion: true, ancho: 1 },
                       { t: h.t, y: h.limn, color: css('--c5'), guion: true, ancho: 1 },
                       { t: h.t, y: h.cmd, color: css('--c4') }], v);
  const k = S.estado.nombres.indexOf(nombre);
  const m = k >= 0 ? S.estado.mando[k] : '';
  $('u-mando').textContent = m === 'corriente_con_modelo' ? '[A]' : m === 'torque_sin_modelo' ? '[N·m]' : '[posición]';
}

// ------------------------------------------------------------ construcción de tablas
function construir(e) {
  // gráficas
  const sg = $('sel-graf'); const prev = sg.value; sg.innerHTML = '';
  e.nombres.forEach((n) => sg.appendChild(el('option', { value: n, text: n })));
  if (e.nombres.includes(prev)) sg.value = prev;

  // mover
  const tm = $('tabla-mover').querySelector('tbody'); tm.innerHTML = '';
  e.nombres.forEach((n, k) => {
    const obj = numInput(`obj-${n}`, null, 1); obj.placeholder = '—';
    const b = el('button', { class: 'btn chico', text: 'Mover sólo ésta' });
    b.onclick = () => moverUna(n);
    tm.appendChild(el('tr', {}, [el('td', { text: n }), el('td', { class: 'num', id: `act-${n}` }),
      el('td', {}, [obj]), el('td', {}, [b])]));
  });

  // perfil
  const tp = $('tabla-perfil').querySelector('tbody'); tp.innerHTML = '';
  e.nombres_perfil.forEach((n, k) => {
    const s = el('select', { id: `est-${n}` });
    for (const o of ['activa', 'bloqueada', 'ausente']) s.appendChild(el('option', { value: o, text: o }));
    s.value = e.estados_perfil[k];
    // brazo serial: si una articulación no está montada, las siguientes tampoco
    s.onchange = () => {
      if (s.value !== 'ausente') return;
      e.nombres_perfil.slice(k + 1).forEach((m) => { $(`est-${m}`).value = 'ausente'; });
    };
    const ka = e.nombres.indexOf(n);
    tp.appendChild(el('tr', { class: e.estados_perfil[k] === 'activa' ? '' : 'inactiva' },
      [el('td', { text: n }), el('td', {}, [s]), el('td', { text: ka >= 0 ? e.mando[ka] : '—' })]));
  });
  const sb = $('sel-brazo'); sb.innerHTML = '';
  for (const p of (e.perfiles.length ? e.perfiles : [e.brazo])) sb.appendChild(el('option', { value: p, text: p }));
  sb.value = e.brazo;

  // ganancias
  const tg = $('tabla-ganancias').querySelector('tbody'); tg.innerHTML = '';
  e.nombres.forEach((n, k) => {
    if (e.mando[k] === 'posicion_directa') return;
    const celdas = [el('td', { text: n }), el('td', {}, [el('span', { class: 'modo', id: `modo-${n}` })])];
    for (let p = 0; p < 4; p++) celdas.push(el('td', { class: 'c-polos' }, [numInput(`pol-${n}-${p}`, e.polos[k][p], 0.5)]));
    for (const [c, paso] of [['wn', 0.5], ['zeta', 0.05], ['eps', 0.01], ['zeta_observador', 0.05]])
      celdas.push(el('td', { class: 'c-zeta' }, [numInput(`zt-${n}-${c}`, null, paso)]));
    for (const c of ['beta', 'alfa', 'armadura', 'alfa_gravedad', 'friccion_coulomb', 'limite_mando'])
      celdas.push(el('td', { class: 'c-cal' }, [numInput(`gan-${n}-${c}`, null, 0.01)]));
    celdas.push(el('td', { class: 'num', id: `kap-${n}` }));
    celdas.push(el('td', { class: 'num', id: `sob-${n}` }));
    celdas.push(el('td', { class: 'num', id: `sat-${n}` }));
    const b = el('button', { class: 'btn chico primario', text: 'Aplicar' });
    b.onclick = () => aplicarGanancias(n);
    celdas.push(el('td', {}, [b]));
    tg.appendChild(el('tr', {}, celdas));
  });
  fijarVista(S.vista);
  leerConfig();

  // autotune
  const aa = $('at-arts'); aa.innerHTML = '';
  e.nombres.forEach((n, k) => {
    if (e.mando[k] !== 'corriente_con_modelo') return;
    const c = el('input', { type: 'checkbox', id: `at-${n}` }); c.checked = true;
    aa.appendChild(el('label', { class: 'chk' }, [c, document.createTextNode(n)]));
  });
  S.sucias.clear();
}

// ------------------------------------------------------------ actualización
function insignia(id, texto, clase) { const b = $(id); b.textContent = texto; b.className = 'insignia ' + (clase || ''); }

function actualizar(e) {
  S.estado = e;
  insignia('b-conexion', e.conectado ? 'conectado' : 'sin datos del control', e.conectado ? 'ok' : 'malo');
  if (!e.nombres) return;
  const clave = e.brazo + '|' + e.nombres.join(',') + '|' + e.estados_perfil.join(',') + '|' + e.mando.join(',');
  if (clave !== S.clave) { S.clave = clave; construir(e); }
  $('brazo').textContent = e.brazo; $('plataforma').textContent = e.plataforma;
  const claseEstado = { habilitado: 'ok', deshabilitado: 'aviso', esperando_medicion: 'aviso', paro: 'malo', falla: 'malo' }[e.estado] || '';
  insignia('b-estado', e.estado, claseEstado);
  insignia('b-watchdog', 'watchdog ' + e.watchdog, e.watchdog === 'ok' ? 'ok' : e.watchdog === 'congelar' ? 'aviso' : 'malo');
  const nsat = (e.saturado || []).filter(Boolean).length;
  insignia('b-saturacion', nsat ? `saturando (${nsat})` : 'sin saturación', nsat ? 'aviso' : 'ok');
  insignia('b-calibracion', e.modo_calibracion ? 'calibración ON' : 'calibración off', e.modo_calibracion ? 'aviso' : '');
  $('motivo').textContent = (e.estado === 'paro' || e.estado === 'falla') ? `${e.estado.toUpperCase()}: ${e.motivo}` : '';
  $('btn-paro').classList.toggle('activo', !!e.paro_emergencia);
  $('btn-liberar').disabled = !e.paro_emergencia;
  $('btn-habilitar').disabled = e.estado === 'habilitado' || e.estado === 'paro';
  $('btn-deshabilitar').disabled = e.estado !== 'habilitado';
  const libre = (id) => document.activeElement !== $(id);
  if (libre('chk-calibracion')) $('chk-calibracion').checked = !!e.modo_calibracion;
  if (libre('chk-supervisor')) $('chk-supervisor').checked = !!e.supervisor_habilitado;
  fijarSiLibre('in-pct-def', e.velocidad_pct_defecto, 0);
  fijarSiLibre('in-dur-def', e.duracion_defecto, 2);
  if (e.modo_trayectoria && !S.sucias.has('def-modo') && libre('def-modo')) $('def-modo').value = e.modo_trayectoria;
  if (S.modoMovAuto && e.modo_trayectoria) { S.modoMovAuto = false; fijarModoMov(e.modo_trayectoria, false); }
  if (!S.durIni && e.duracion_defecto) { S.durIni = true; if (!S.sucias.has('in-dur')) $('in-dur').value = numTexto(e.duracion_defecto, 2); }
  if (e.shaper) { if (!S.sucias.has('sh-tipo') && libre('sh-tipo')) $('sh-tipo').value = e.shaper.tipo;
    fijarSiLibre('sh-f', e.shaper.frecuencia_hz, 3); fijarSiLibre('sh-z', e.shaper.amortiguamiento, 3); }

  e.nombres.forEach((n, k) => {
    const a = $(`act-${n}`); if (a) a.textContent = fmt(e.posicion_deg[k]) + '°';
    if (e.mando[k] === 'posicion_directa') return;
    const md = $(`modo-${n}`);
    if (md && e.sintonia) {
      const m = e.sintonia[k] || 'polos';
      md.textContent = m === 'zeta' ? 'ζ, ωn, ε' : 'polos';
      md.classList.toggle('igual', m === S.vista);
    }
    for (let p = 0; p < 4; p++) {
      fijarSiLibre(`pol-${n}-${p}`, e.polos[k][p], 3);
      const im = e.polos_imag ? e.polos_imag[k][p] : 0; const i = $(`pol-${n}-${p}`);
      if (i) i.title = Math.abs(im) > 1e-9 ? `polo complejo: −${fmt(e.polos[k][p])} ${im >= 0 ? '+' : '−'} ${fmt(Math.abs(im))}j` : '';
    }
    if (e.zeta) {
      fijarSiLibre(`zt-${n}-wn`, e.wn_sintonia[k], 3); fijarSiLibre(`zt-${n}-zeta`, e.zeta[k], 3);
      fijarSiLibre(`zt-${n}-eps`, e.eps[k], 4); fijarSiLibre(`zt-${n}-zeta_observador`, e.zeta_observador[k], 3);
    }
    for (const c of ['beta', 'alfa', 'armadura', 'alfa_gravedad', 'friccion_coulomb']) fijarSiLibre(`gan-${n}-${c}`, e[c][k], 4);
    fijarSiLibre(`gan-${n}-limite_mando`, e.limite_mando[k], 3);
    const kap = $(`kap-${n}`); if (kap) kap.textContent = fmt(e.kappa[k]);
    const sob = $(`sob-${n}`); if (sob) sob.textContent = fmt(e.sobreimpulso_deg[k]) + '°';
    const sat = $(`sat-${n}`); if (sat) sat.textContent = e.saturaciones[k];
  });

  const at = e.autotune || {};
  const prog = e.autotune_activo ? e.autotune_progreso : (at.progreso || 0);
  $('at-barra').style.width = `${Math.round(100 * prog)}%`;
  $('at-etapa').textContent = e.autotune_activo ? e.autotune_etapa : (at.mensaje || '');
  if (at.resultado && $('at-resultado').textContent !== at.resultado) $('at-resultado').textContent = at.resultado;
  $('btn-autotune').disabled = !!e.autotune_activo || e.estado !== 'habilitado';
  if (e.movimiento && !e.movimiento.activo && e.movimiento.mensaje) {
    const sob = (e.movimiento.sobreimpulso_deg || []).map((v) => fmt(v)).join(', ');
    $('nota-mov').textContent = e.movimiento.mensaje + (sob ? ` | sobreimpulso [°]: ${sob}` : '');
  }
  const ev = $('eventos'); const lista = (e.eventos || []).slice().reverse();
  if (ev.dataset.k !== lista.join('|')) {
    ev.dataset.k = lista.join('|'); ev.innerHTML = '';
    for (const t of lista) ev.appendChild(el('li', { text: t }));
  }
  guardarHistorial(e);
}

// ------------------------------------------------------------ acciones
function moverUna(n) {
  const v = $(`obj-${n}`).value;
  if (v === '') { toast('Escribe el objetivo de ' + n, true); return; }
  comando('mover', Object.assign({ posiciones_deg: { [n]: Number(v) } }, ritmo()));
}
function moverTodas() {
  const pos = {};
  for (const n of S.estado.nombres) { const v = $(`obj-${n}`).value; if (v !== '') pos[n] = Number(v); }
  if (!Object.keys(pos).length) { toast('No hay objetivos escritos', true); return; }
  comando('mover', Object.assign({ posiciones_deg: pos }, ritmo()));
}
// Lo que se manda con cada movimiento según la pestaña: % o segundos.
function ritmo() {
  return S.modoMov === 'tiempo' ? { duracion_s: Number($('in-dur').value) }
    : { velocidad_pct: Number($('in-pct').value) };
}
function fijarModoMov(m, recordar = true) {
  S.modoMov = m;
  if (recordar) { try { localStorage.setItem('modo-mover', m); } catch (err) { /* sin almacenamiento */ } }
  const t = $('pest-mover').closest('section');
  t.classList.remove('modo-velocidad', 'modo-tiempo'); t.classList.add(`modo-${m}`);
  for (const b of document.querySelectorAll('#pest-mover .pestana')) {
    const on = b.dataset.modo === m; b.classList.toggle('activa', on); b.setAttribute('aria-selected', on);
  }
}
// Cualquier tarjeta se pliega con un clic en su título (se recuerda).
const PLEGADAS_DEFECTO = ['Conformador (base elástica)', 'Autotuning', 'Configuración de la plataforma'];
function prepararPlegables() {
  let guardado = null;
  try { guardado = JSON.parse(localStorage.getItem('plegadas') || 'null'); } catch (err) { /* sin almacenamiento */ }
  const plegadas = new Set(guardado || PLEGADAS_DEFECTO);
  for (const sec of document.querySelectorAll('section.tarjeta')) {
    const h = sec.querySelector('.cabecera h2'); if (!h) continue;
    const clave = h.firstChild.textContent.trim();
    h.classList.add('plegable'); h.title = 'Clic para plegar o desplegar';
    sec.classList.toggle('plegada', plegadas.has(clave));
    h.addEventListener('click', (ev) => {
      if (ev.target.closest('.insignia')) return;
      sec.classList.toggle('plegada');
      sec.classList.contains('plegada') ? plegadas.add(clave) : plegadas.delete(clave);
      try { localStorage.setItem('plegadas', JSON.stringify([...plegadas])); } catch (err) { /* sin almacenamiento */ }
      if (!sec.classList.contains('plegada')) window.dispatchEvent(new Event('resize'));
    });
  }
}
// Pestañas: "polos" y "zeta" eligen con qué se programa la sintonía; "cal"
// muestra β, α, armadura, α_g, fricción y límite. Cada pestaña aplica sólo
// lo suyo.
function fijarVista(v) {
  S.vista = v;
  try { localStorage.setItem('vista-ganancias', v); } catch (err) { /* sin almacenamiento */ }
  const t = $('tabla-ganancias');
  t.classList.remove('vista-polos', 'vista-zeta', 'vista-cal'); t.classList.add(`vista-${v}`);
  const tarjeta = t.closest('section');
  tarjeta.classList.remove('vista-polos-nota', 'vista-zeta-nota', 'vista-cal-nota'); tarjeta.classList.add(`vista-${v}-nota`);
  for (const b of document.querySelectorAll('#pest-ganancias .pestana')) {
    const on = b.dataset.vista === v; b.classList.toggle('activa', on); b.setAttribute('aria-selected', on);
  }
  if (S.estado) S.estado.nombres.forEach((n, k) => {
    const md = $(`modo-${n}`); if (md && S.estado.sintonia) md.classList.toggle('igual', (S.estado.sintonia[k] || 'polos') === v);
  });
}
async function aplicarGanancias(n) {
  const p = {}; const v = S.vista;
  if (v === 'polos') {
    const pol = [];
    for (let k = 0; k < 4; k++) pol.push(Number($(`pol-${n}-${k}`).value));
    if (pol.some((x) => !(x > 0))) { toast('Los 4 polos deben ser positivos', true); return; }
    p[`${n}.sintonia`] = 'polos'; p[`${n}.polos`] = pol;
  } else if (v === 'zeta') {
    const z = {};
    for (const c of ['wn', 'zeta', 'eps', 'zeta_observador']) z[c] = Number($(`zt-${n}-${c}`).value || 0);
    if (!(z.wn > 0)) { toast('ωn debe ser > 0', true); return; }
    if (!(z.eps > 0 && z.eps <= 1)) { toast('ε debe estar en (0, 1]', true); return; }
    if (!(z.zeta >= 0.2 && z.zeta <= 5)) { toast('ζ debe estar en [0.2, 5]', true); return; }
    p[`${n}.sintonia`] = 'zeta';
    for (const c of ['wn', 'zeta', 'eps', 'zeta_observador']) p[`${n}.${c}`] = z[c];
  } else {
    for (const c of ['beta', 'alfa', 'armadura', 'alfa_gravedad', 'friccion_coulomb', 'limite_mando']) {
      const x = $(`gan-${n}-${c}`).value; if (x !== '') p[`${n}.${c}`] = Number(x);
    }
  }
  const pref = { polos: 'pol-', zeta: 'zt-', cal: 'gan-' }[v];
  const r = await comando('parametros', { parametros: p });
  if (r.ok) for (const id of [...S.sucias]) if (id.startsWith(`${pref}${n}-`)) S.sucias.delete(id);
}
function limpiarSucias(prefijos) {
  for (const id of [...S.sucias]) if (prefijos.some((p) => id.startsWith(p))) S.sucias.delete(id);
}

// ------------------------------------------------------------ plataforma
const CAMPOS_INTERFAZ = [
  ['topico_estado', 'Tópico de estado', 'text'], ['qos_estado_best_effort', 'QoS best effort (micro-ROS)', 'bool'],
  ['tipo_mando', 'Tipo de mando', 'tipo_mando'], ['patron_topico_mando', 'Patrón tópico de mando', 'text'],
  ['topico_mando_unico', 'Tópico de mando único', 'text'], ['topico_imu', 'Tópico IMU', 'text'],
  ['usar_stamp_medicion', 'Usar stamp de la medición', 'bool'],
];
function campoPlat(id, tipo, valor, opciones) {
  let i;
  if (tipo === 'bool') { i = el('input', { type: 'checkbox', id }); i.checked = !!valor; }
  else if (opciones) {
    i = el('select', { id });
    for (const o of opciones) i.appendChild(el('option', { value: String(o), text: String(o) }));
    i.value = String(valor);
  } else if (tipo === 'num') { i = el('input', { type: 'number', step: '0.0001', id }); i.value = numTexto(valor, 4); }
  else { i = el('input', { type: 'text', id }); i.value = valor ?? ''; }
  i.dataset.original = tipo === 'bool' ? String(!!valor) : i.value;
  const marcar = () => i.classList.toggle('editado', valorCampo(i) !== i.dataset.original);
  i.addEventListener('input', marcar); i.addEventListener('change', marcar);
  return i;
}
function valorCampo(i) { return i.type === 'checkbox' ? String(i.checked) : i.value; }
function pintarConfig(cfg) {
  S.config = cfg;
  insignia('b-plat-conf', cfg.plataforma, cfg.plataforma === 'real' ? 'aviso' : 'ok');
  const fi = $('plat-interfaz'); fi.innerHTML = '';
  for (const [k, txt, tipo] of CAMPOS_INTERFAZ) {
    const ops = tipo === 'tipo_mando' ? cfg.opciones.tipo_mando : null;
    const i = campoPlat(`pi-${k}`, tipo === 'tipo_mando' ? 'text' : tipo, cfg.interfaz[k], ops);
    fi.appendChild(tipo === 'bool' ? el('label', { class: 'chk' }, [i, document.createTextNode(txt)])
      : el('label', {}, [document.createTextNode(txt), i]));
  }
  const tb = $('tabla-plataforma').querySelector('tbody'); tb.innerHTML = '';
  for (const a of cfg.articulaciones) {
    const id = (k) => `pa-${a.nombre}-${k}`;
    tb.appendChild(el('tr', { class: a.estado === 'activa' ? '' : 'inactiva' }, [
      el('td', { text: a.nombre }), el('td', { text: a.estado }), el('td', { text: a.mando }),
      el('td', {}, [campoPlat(id('signo'), 'num', a.signo, ['1', '-1'])]),
      el('td', {}, [campoPlat(id('encoder'), 'text', a.encoder, cfg.opciones.encoder)]),
      el('td', {}, [campoPlat(id('offset_absoluto'), 'num', a.offset_absoluto)]),
      el('td', {}, [campoPlat(id('postura_encendido_urdf'), 'num', a.postura_encendido_urdf)]),
      el('td', {}, [campoPlat(id('nombre_en_estado'), 'text', a.nombre_en_estado)]),
      el('td', {}, [campoPlat(id('nombre_en_mando'), 'text', a.nombre_en_mando)]),
      el('td', {}, [campoPlat(id('unidad_posicion'), 'text', a.unidad_posicion, cfg.opciones.unidad_posicion)]),
    ]));
  }
  // el select de signo usa '1'/'-1'
  for (const a of cfg.articulaciones) {
    const s = $(`pa-${a.nombre}-signo`); s.value = a.signo < 0 ? '-1' : '1'; s.dataset.original = s.value;
  }
  const ar = cfg.archivos;
  $('plat-archivos').textContent = `Perfil del brazo:  ${ar.perfil}\nPlataforma:        ${ar.plataforma}\n`
    + `Ajustes guardados: ${ar.ajustes}${ar.ajustes_cargados ? '' : '  (todavía no existe: arranca con valores de fábrica)'}`
    + (ar.ajustes_cargados && ar.ajustes_cargados !== ar.ajustes ? `\nCargado al arrancar: ${ar.ajustes_cargados}` : '');
  $('nota-ajustes').textContent = `Plataforma "${cfg.plataforma}": las ganancias se guardan en ${ar.ajustes} y el control arranca con ellas`
    + (ar.ajustes_cargados ? '.' : ' (aún no hay nada guardado: arrancó con los valores de fábrica).');
}
async function leerConfig() {
  const r = await comando('config_leer');
  if (r.ok && r.config) pintarConfig(r.config);
}
async function guardarPlataforma(reiniciar) {
  const cfg = S.config; if (!cfg) return;
  const datos = { interfaz: {}, articulaciones: {} };
  for (const [k] of CAMPOS_INTERFAZ) {
    const i = $(`pi-${k}`); if (valorCampo(i) === i.dataset.original) continue;
    datos.interfaz[k] = i.type === 'checkbox' ? i.checked : i.value;
  }
  for (const a of cfg.articulaciones) {
    for (const k of ['signo', 'encoder', 'offset_absoluto', 'postura_encendido_urdf', 'nombre_en_estado', 'nombre_en_mando', 'unidad_posicion']) {
      const i = $(`pa-${a.nombre}-${k}`); if (valorCampo(i) === i.dataset.original) continue;
      const v = ['signo', 'offset_absoluto', 'postura_encendido_urdf'].includes(k) ? Number(i.value) : i.value;
      (datos.articulaciones[a.nombre] = datos.articulaciones[a.nombre] || {})[k] = v;
    }
  }
  if (!Object.keys(datos.interfaz).length && !Object.keys(datos.articulaciones).length) { toast('No hay cambios en la plataforma', true); return; }
  const r = await comando('guardar_plataforma', { datos, reiniciar });
  if (r.ok) { S.clave = ''; leerConfig(); }
}

function estadosTexto() {
  return S.estado.nombres_perfil.map((n) => `${n}=${$(`est-${n}`).value}`).join(',');
}

function enlazar() {
  let vista = null;
  try { vista = localStorage.getItem('vista-ganancias'); } catch (err) { /* sin almacenamiento */ }
  fijarVista(['polos', 'zeta', 'cal'].includes(vista) ? vista : 'polos');
  for (const b of document.querySelectorAll('#pest-ganancias .pestana')) b.onclick = () => fijarVista(b.dataset.vista);
  $('btn-paro').onclick = () => comando('paro');
  $('btn-liberar').onclick = () => { if (confirm('¿Liberar el paro de emergencia? El control queda deshabilitado.')) comando('liberar'); };
  $('btn-habilitar').onclick = () => comando('habilitar');
  $('btn-deshabilitar').onclick = () => comando('deshabilitar');
  $('btn-mover').onclick = moverTodas;
  $('btn-cero').onclick = () => {
    const pos = {}; S.estado.nombres.forEach((n) => { pos[n] = 0; });
    comando('mover', Object.assign({ posiciones_deg: pos }, ritmo()));
  };
  $('btn-copiar').onclick = () => S.estado.nombres.forEach((n, k) => { $(`obj-${n}`).value = fmt(S.estado.posicion_deg[k]); });
  $('btn-cancelar-mov').onclick = () => comando('cancelar_movimiento');
  $('btn-perfil').onclick = () => {
    if (!confirm('Aplicar el perfil reinicia el control y fija el cero en la postura actual. ¿Continuar?')) return;
    comando('perfil', { brazo: $('sel-brazo').value === S.estado.brazo ? '' : $('sel-brazo').value, montadas: 0, estados: estadosTexto() });
    S.clave = '';
  };
  $('btn-tray-def').onclick = async () => {
    const r = await comando('parametros', { parametros: {
      'trayectoria.modo': $('def-modo').value,
      velocidad_pct_defecto: Number($('in-pct-def').value),
      'trayectoria.duracion_defecto': Number($('in-dur-def').value) } });
    if (r.ok) limpiarSucias(['in-pct-def', 'in-dur-def', 'def-modo']);
  };
  let modoGuardado = null;
  try { modoGuardado = localStorage.getItem('modo-mover'); } catch (err) { /* sin almacenamiento */ }
  // Sin preferencia guardada, la pestaña sigue al modo predeterminado del control.
  if (['velocidad', 'tiempo'].includes(modoGuardado)) fijarModoMov(modoGuardado, false);
  else { fijarModoMov('velocidad', false); S.modoMovAuto = true; }
  for (const b of document.querySelectorAll('#pest-mover .pestana')) b.onclick = () => fijarModoMov(b.dataset.modo);
  for (const i of ['in-pct', 'in-dur']) $(i).addEventListener('input', () => S.sucias.add(i));
  prepararPlegables();
  $('chk-supervisor').onchange = (ev) => comando('parametros', { parametros: { 'supervisor.habilitado': ev.target.checked } });
  $('btn-shaper').onclick = async () => {
    const r = await comando('parametros', { parametros: {
      'shaper.tipo': $('sh-tipo').value, 'shaper.frecuencia_hz': Number($('sh-f').value),
      'shaper.amortiguamiento': Number($('sh-z').value) } });
    if (r.ok) ['sh-tipo', 'sh-f', 'sh-z'].forEach((i) => S.sucias.delete(i));
  };
  for (const i of ['sh-tipo', 'sh-f', 'sh-z', 'in-pct-def', 'in-dur-def', 'def-modo']) $(i).addEventListener('input', () => S.sucias.add(i));
  $('btn-modo').onclick = async () => { const r = await comando('estimar_modo'); $('salida-modo').textContent = r.mensaje || ''; };
  $('btn-autotune').onclick = () => {
    const arts = S.estado.nombres.filter((n) => $(`at-${n}`) && $(`at-${n}`).checked);
    if (!arts.length) { toast('Elige al menos una articulación', true); return; }
    if (!confirm(`El autotuning moverá ${arts.join(', ')} en 3 posturas. Revisa que no haya obstáculos. ¿Iniciar?`)) return;
    comando('autotune', { articulaciones: arts, aplicar: $('at-aplicar').checked, guardar: $('at-guardar').checked });
  };
  $('btn-cancelar-at').onclick = () => comando('cancelar_autotune');
  $('chk-calibracion').onchange = (ev) => comando('calibracion', { activar: ev.target.checked });
  $('btn-guardar').onclick = async () => {
    const r = await comando('guardar', { incluir_estados: $('chk-estados').checked });
    if (r.ok) leerConfig();
  };
  $('btn-restaurar').onclick = async () => {
    if (!confirm('¿Descartar los cambios sin guardar y volver a las últimas ganancias guardadas?')) return;
    const r = await comando('restaurar_guardadas');
    if (r.ok) limpiarSucias(['pol-', 'zt-', 'gan-', 'in-pct-def', 'in-dur-def', 'def-modo', 'sh-']);
  };
  $('btn-fabrica').onclick = async () => {
    if (!confirm('¿Cargar los valores de fábrica (perfil + plataforma)? No se borra lo guardado: si no te gustan, usa "Volver a las guardadas".')) return;
    const r = await comando('restaurar_fabrica');
    if (r.ok) limpiarSucias(['pol-', 'zt-', 'gan-', 'in-pct-def', 'in-dur-def', 'def-modo', 'sh-']);
  };
  $('btn-plat-recargar').onclick = () => leerConfig();
  $('btn-plat-guardar').onclick = () => guardarPlataforma(false);
  $('btn-plat-reiniciar').onclick = () => {
    if (!confirm('Se guardan los cambios y se REINICIA el control (queda deshabilitado). ¿Continuar?')) return;
    guardarPlataforma(true);
  };
  $('btn-pausa').onclick = (ev) => { S.pausa = !S.pausa; ev.target.textContent = S.pausa ? 'Reanudar' : 'Pausar'; };
  document.addEventListener('keydown', (ev) => {
    if (ev.code === 'Space' && !['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
      ev.preventDefault(); comando('paro');
    }
  });
}

// ------------------------------------------------------------ arranque
function conectar() {
  const es = new EventSource('/api/stream');
  es.onmessage = (m) => { try { actualizar(JSON.parse(m.data)); } catch (e) { console.error(e); } };
  es.onerror = () => { insignia('b-conexion', 'reconectando…', 'malo'); };
}
setInterval(() => { fetch('/api/heartbeat', { method: 'POST', body: '{}' }).catch(() => {}); }, 200);
setInterval(redibujar, 100);
enlazar();
conectar();
