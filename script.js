const g = 9.81;
let chart = null;
const defaultConfig = {
  M: 0.022,
  m: 0.0025,
  L1: 0.2,
  L2: 0.12,
  Ls: 0.1,
  thetaR: 45,
  pivotH: 0.3
};

function getVals() {
  return {
    M: +document.getElementById('sM').value,
    m: +document.getElementById('sm').value,
    L1: +document.getElementById('sL1').value,
    L2: +document.getElementById('sL2').value,
    Ls: +document.getElementById('sLs').value,
    thetaR: +document.getElementById('sTr').value
  };
}

function resetDefaults() {
  document.getElementById('sM').value = defaultConfig.M;
  document.getElementById('sm').value = defaultConfig.m;
  document.getElementById('sL1').value = defaultConfig.L1;
  document.getElementById('sL2').value = defaultConfig.L2;
  document.getElementById('sLs').value = defaultConfig.Ls;
  document.getElementById('sTr').value = defaultConfig.thetaR;
  update();
}

function compute(v) {
  const theta_swing = Math.PI * 0.75;
  const I = v.M * v.L2 * v.L2 + v.m * v.L1 * v.L1;
  const tau = v.M * g * v.L2 - v.m * g * v.L1;
  if (tau <= 0) return null;
  const omega = Math.sqrt(Math.max(0, 2 * tau * theta_swing / I));
  const T_sling = 2 * Math.PI * Math.sqrt(v.Ls / g);
  const v0 = omega * (v.L1 + v.Ls);
  const thetaLaunch = v.thetaR * Math.PI / 180;
  const vx = v0 * Math.cos(thetaLaunch);
  const vy = v0 * Math.sin(thetaLaunch);
  const T_flight = 2 * vy / g;
  const R = vx * T_flight;
  const H = vy * vy / (2 * g);
  const Ep_in = v.M * g * v.L2 * theta_swing;
  const Ek_out = 0.5 * v.m * v0 * v0;
  const eta = Ep_in > 0 ? (Ek_out / Ep_in) * 100 : 0;
  const impulse = v.m * v0;
  return { tau, omega, T_sling, v0, thetaLaunch, T_flight, R, H, eta, impulse };
}

function drawTrebuchet(v, res) {
  const svg = document.getElementById('tsvg');
  const cx = 150, groundY = 190, pivotX = cx;
  const pivotPx = 100;
  const pivotY = groundY - pivotPx;
  const scale = 80;
  const minArmPx = 24;
  const minSlingPx = 18;
  const L1px = Math.max(Math.min(v.L1 * scale, 110), minArmPx);
  const L2px = Math.max(Math.min(v.L2 * scale, 60), minArmPx * 0.6);
  const Lspx = Math.max(Math.min(v.Ls * scale, 45), minSlingPx);
  const angle1 = -v.thetaR * Math.PI / 180;
  const arm1ex = pivotX + L1px * Math.cos(angle1);
  const arm1ey = pivotY + L1px * Math.sin(angle1);
  const arm2ex = pivotX - L2px * Math.cos(angle1);
  const arm2ey = pivotY - L2px * Math.sin(angle1);
  const cwSize = Math.max(Math.min(6 + (v.M / 50) * 18, 24), 8);
  const projR = Math.max(Math.min(3 + (v.m / 2) * 5, 8), 4);
  const slingTipX = arm1ex + Lspx * Math.sin(0.4);
  const slingTipY = arm1ey + Lspx * Math.cos(0.4);
  const frameCol = '#202020';
  const armCol = '#00a075';
  const textCol = '#202020';
  const slingCol = '#202020';

  svg.innerHTML = `
    <line x1="${cx-55}" y1="${groundY}" x2="${cx+55}" y2="${groundY}" stroke="${frameCol}" stroke-width="2" stroke-linecap="round"/>
    <line x1="${cx-40}" y1="${groundY}" x2="${pivotX-10}" y2="${pivotY+10}" stroke="${frameCol}" stroke-width="3" stroke-linecap="round"/>
    <line x1="${cx+40}" y1="${groundY}" x2="${pivotX+10}" y2="${pivotY+10}" stroke="${frameCol}" stroke-width="3" stroke-linecap="round"/>
    <line x1="${cx-40}" y1="${groundY-20}" x2="${cx+40}" y2="${groundY-20}" stroke="${frameCol}" stroke-width="2" stroke-linecap="round" opacity="0.4"/>
    <line x1="${arm1ex}" y1="${arm1ey}" x2="${arm2ex}" y2="${arm2ey}" stroke="${armCol}" stroke-width="5" stroke-linecap="round"/>
    <circle cx="${pivotX}" cy="${pivotY}" r="6" fill="${armCol}" stroke="#ffffff" stroke-width="1.5"/>
    <circle cx="${pivotX}" cy="${pivotY}" r="2.5" fill="#ffffff"/>
    <rect x="${arm2ex - cwSize/2}" y="${arm2ey}" width="${cwSize}" height="${cwSize*0.9}" rx="3" fill="${armCol}" opacity="0.9"/>
    <line x1="${arm1ex}" y1="${arm1ey}" x2="${slingTipX}" y2="${slingTipY}" stroke="${slingCol}" stroke-width="2" stroke-dasharray="4 2"/>
    <circle cx="${slingTipX}" cy="${slingTipY}" r="${projR}" fill="${armCol}" stroke="#202020" stroke-width="1"/>
    <text x="${arm2ex - cwSize/2 - 4}" y="${arm2ey + cwSize*0.45}" text-anchor="end" font-size="9" fill="${textCol}">${v.M.toFixed(3)}kg</text>
    <text x="${slingTipX + projR + 3}" y="${slingTipY + 4}" font-size="9" fill="${textCol}">${v.m.toFixed(4)}kg</text>
    <text x="${cx-28}" y="${pivotY-8}" font-size="8" fill="${textCol}">L₁=${v.L1.toFixed(2)}m</text>
    <text x="${cx+4}" y="${pivotY-8}" font-size="8" fill="${textCol}">L₂=${v.L2.toFixed(2)}m</text>
    ${res ? `<text x="10" y="20" font-size="8" fill="${textCol}">v₀=${res.v0.toFixed(1)}m/s</text><text x="10" y="32" font-size="8" fill="${textCol}">θ=${v.thetaR}°</text>` : ''}
  `;
}

function drawTrajectory(v, res) {
  const canvas = document.getElementById('traj');
  if (!res || res.R <= 0) { if (chart) { chart.destroy(); chart = null; } return; }
  const pts = [];
  const vx = res.v0 * Math.cos(res.thetaLaunch);
  const vy = res.v0 * Math.sin(res.thetaLaunch);
  for (let i = 0; i <= 60; i++) {
    const t = (i / 60) * res.T_flight;
    pts.push({ x: parseFloat((vx * t).toFixed(2)), y: parseFloat((vy * t - 0.5 * g * t * t).toFixed(2)) });
  }
  const labelColor = '#202020';
  const gridColor = 'rgba(32,32,32,0.18)';
  if (chart) { chart.destroy(); chart = null; }
  chart = new Chart(canvas, {
    type: 'scatter',
    data: { datasets: [{ label: 'Trajectory', data: pts, showLine: true, borderColor: '#00a075', backgroundColor: 'transparent', pointRadius: 0, borderWidth: 3, tension: 0.35 }] },
    options: {
      responsive: true, maintainAspectRatio: false,
      plugins: { legend: { display: false } },
      scales: {
        x: { title: { display: true, text: 'Range (m)', color: labelColor, font: { size: 10 } }, ticks: { color: labelColor, font: { size: 9 }, maxTicksLimit: 6 }, grid: { color: gridColor }, min: 0 },
        y: { title: { display: true, text: 'Height (m)', color: labelColor, font: { size: 10 } }, ticks: { color: labelColor, font: { size: 9 }, maxTicksLimit: 5 }, grid: { color: gridColor }, min: 0 }
      }
    }
  });
}

function update() {
  const v = getVals();
  document.getElementById('lM').textContent = v.M.toFixed(3) + ' kg';
  document.getElementById('lm').textContent = v.m.toFixed(4) + ' kg';
  document.getElementById('lL1').textContent = v.L1.toFixed(2) + ' m';
  document.getElementById('lL2').textContent = v.L2.toFixed(2) + ' m';
  document.getElementById('lLs').textContent = v.Ls.toFixed(2) + ' m';
  document.getElementById('lTr').textContent = v.thetaR + '°';
  const res = compute(v);
  drawTrebuchet(v, res);
  if (!res) {
    ['rv0','rth','rR','rH','rT','rEff'].forEach(id => document.getElementById(id).textContent = '—');
    ['fTau','fOmega','fTsling','fV0','fImpulse','fRange','fEta'].forEach(id => document.getElementById(id).textContent = '—');
    drawTrajectory(v, null); return;
  }
  document.getElementById('rv0').textContent = res.v0.toFixed(2);
  document.getElementById('rth').textContent = v.thetaR;
  document.getElementById('rR').textContent = res.R.toFixed(2);
  document.getElementById('rH').textContent = res.H.toFixed(2);
  document.getElementById('rT').textContent = res.T_flight.toFixed(2);
  document.getElementById('rEff').textContent = res.eta.toFixed(1);
  document.getElementById('fTau').textContent = res.tau.toFixed(2) + ' N·m';
  document.getElementById('fOmega').textContent = res.omega.toFixed(2) + ' rad/s';
  document.getElementById('fTsling').textContent = res.T_sling.toFixed(3) + ' s';
  document.getElementById('fV0').textContent = res.v0.toFixed(2) + ' m/s';
  document.getElementById('fImpulse').textContent = res.impulse.toFixed(3) + ' N·s';
  document.getElementById('fRange').textContent = res.R.toFixed(2) + ' m';
  document.getElementById('fEta').textContent = res.eta.toFixed(1) + ' %';
  drawTrajectory(v, res);
}

['sM','sm','sL1','sL2','sLs','sTr'].forEach(id => document.getElementById(id).addEventListener('input', update));
document.getElementById('resetBtn').addEventListener('click', resetDefaults);
update();
