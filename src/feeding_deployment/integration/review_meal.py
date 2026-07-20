#!/usr/bin/env python3
"""Meal review tool -- standalone web page on port 8082.

After a meal, reopen a day's researcher log to edit the note on each
intervention / explanation / note, add entries you didn't capture live, and
write free-form end-of-meal notes and your own thoughts. Everything is saved to
a SEPARATE ``researcher_review.json`` in the day directory -- the original
append-only ``researcher_events.jsonl`` written by researcher_timer.py is never
modified, so the raw capture stays intact for audit while the review file holds
your edited / annotated version.

The first time a session is opened the review is seeded from the original marks
(every interval with its times and note); after that it loads whatever you last
saved. Removing an entry keeps it in the review file tombstoned (``deleted``),
so nothing is silently lost there either.

Run on demand (deliberately NOT part of launch_app, since review happens after
the meal, with the live timer no longer running):

    python review_meal.py                 # then open http://192.168.1.2:8082

Depends only on Flask + the interval-format helpers in compute_feeding_time.py,
matching researcher_timer.py: no roscore, no rosbridge, no run.py.
"""

from __future__ import annotations

import argparse
import json
import logging
import re
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from flask import Flask, Response, jsonify, request

try:
    from feeding_deployment.integration.compute_feeding_time import (
        INTERVAL_KINDS, LOG_ROOT, RESEARCHER_EVENTS_FILENAME,
        load_researcher_intervals, researcher_meal_window)
except ImportError:  # run directly as `python review_meal.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from compute_feeding_time import (
        INTERVAL_KINDS, LOG_ROOT, RESEARCHER_EVENTS_FILENAME,
        load_researcher_intervals, researcher_meal_window)

PORT = 8082
REVIEW_FILENAME = "researcher_review.json"
REVIEW_SCHEMA = "researcher_review/1"
_USER_RE = re.compile(r"^[A-Za-z0-9._-]+$")

app = Flask(__name__)
_lock = threading.Lock()  # guards the read-modify-write of a review file


# -- helpers ------------------------------------------------------------------

def _stamp() -> dict:
    now = time.time()
    return {"epoch": now, "iso": datetime.fromtimestamp(now).isoformat()}


def _valid_day_dir(path: Path) -> bool:
    """True iff path is LOG_ROOT/<user>/day_NN (no traversal outside the root)."""
    try:
        path = path.resolve()
        return (path.parent.parent == LOG_ROOT.resolve()
                and re.fullmatch(r"day_\d+", path.name) is not None
                and _USER_RE.fullmatch(path.parent.name) is not None)
    except OSError:
        return False


def _list_sessions(limit: int = 50) -> list[dict]:
    """Existing day dirs, most recently active first, flagged if already reviewed."""
    sessions = []
    for day_dir in LOG_ROOT.glob("*/day_*"):
        if not day_dir.is_dir() or not _valid_day_dir(day_dir):
            continue
        mtime = day_dir.stat().st_mtime
        for name in ("metadata.json", RESEARCHER_EVENTS_FILENAME, REVIEW_FILENAME):
            try:
                mtime = max(mtime, (day_dir / name).stat().st_mtime)
            except OSError:
                pass
        sessions.append({
            "dir": str(day_dir),
            "user": day_dir.parent.name,
            "day": int(day_dir.name.split("_")[1]),
            "label": f"{day_dir.parent.name} / {day_dir.name}",
            "reviewed": (day_dir / REVIEW_FILENAME).exists(),
            "mtime": mtime,
        })
    sessions.sort(key=lambda s: s["mtime"], reverse=True)
    return sessions[:limit]


def _num_or_none(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _seed_from_original(day_dir: Path) -> dict:
    """Build a fresh review body from the untouched researcher_events.jsonl."""
    intervals, _ = load_researcher_intervals(day_dir)
    m_start, m_end = researcher_meal_window(day_dir)
    return {
        "meal": {"start": m_start, "end": m_end},
        "entries": [{
            "id": iv["id"],
            "kind": iv["kind"],
            "start": iv["start"],
            "end": iv["end"],
            "note": iv["note"],
            "origin": "original",
            "deleted": False,
        } for iv in intervals],
        "reflections": "",
    }


def _load_review(day_dir: Path) -> tuple[dict, bool]:
    """(review body, saved_before). Loads researcher_review.json if present, else
    seeds from the original marks. Never writes anything."""
    path = day_dir / REVIEW_FILENAME
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
        return {
            "meal": doc.get("meal") or {"start": None, "end": None},
            "entries": doc.get("entries") or [],
            "reflections": doc.get("reflections") or "",
            "created": doc.get("created"),
            "updated": doc.get("updated"),
        }, True
    except (OSError, json.JSONDecodeError):
        body = _seed_from_original(day_dir)
        body.update(created=None, updated=None)
        return body, False


def _sanitize_entries(raw) -> list[dict]:
    entries = []
    for e in raw if isinstance(raw, list) else []:
        if not isinstance(e, dict):
            continue
        kind = e.get("kind")
        entries.append({
            "id": e.get("id") if isinstance(e.get("id"), int) else None,
            "kind": kind if kind in INTERVAL_KINDS else "note",
            "start": _num_or_none(e.get("start")),
            "end": _num_or_none(e.get("end")),
            "note": str(e.get("note", "")).strip(),
            "origin": "original" if e.get("origin") == "original" else "added",
            "deleted": bool(e.get("deleted", False)),
        })
    return entries


# -- API ----------------------------------------------------------------------

@app.get("/api/review")
def api_get():
    """List sessions, and (with ?dir=) return that session's review body."""
    resp = {"sessions": _list_sessions(), "session": None, "review": None, "saved": False}
    dir_arg = request.args.get("dir")
    if dir_arg:
        day_dir = Path(dir_arg)
        if _valid_day_dir(day_dir) and day_dir.is_dir():
            body, saved = _load_review(day_dir)
            resp["session"] = {"dir": str(day_dir), "user": day_dir.parent.name,
                               "day": int(day_dir.name.split("_")[1])}
            resp["review"] = body
            resp["saved"] = saved
        else:
            return jsonify({"error": "unknown session"}), 400
    return jsonify(resp)


@app.post("/api/review")
def api_save():
    """Write the edited review to researcher_review.json. The original
    researcher_events.jsonl is never opened for writing here."""
    body = request.get_json(silent=True) or {}
    day_dir = Path(str(body.get("dir", "")))
    if not (_valid_day_dir(day_dir) and day_dir.is_dir()):
        return jsonify({"error": "unknown session"}), 400

    meal = body.get("meal") or {}
    with _lock:
        path = day_dir / REVIEW_FILENAME
        created = None
        try:  # keep the original creation stamp across re-saves
            created = json.loads(path.read_text(encoding="utf-8")).get("created")
        except (OSError, json.JSONDecodeError):
            pass
        now = _stamp()
        doc = {
            "schema": REVIEW_SCHEMA,
            "source": RESEARCHER_EVENTS_FILENAME,
            "created": created or now,
            "updated": now,
            "meal": {"start": _num_or_none(meal.get("start")),
                     "end": _num_or_none(meal.get("end"))},
            "entries": _sanitize_entries(body.get("entries")),
            "reflections": str(body.get("reflections", "")).strip(),
        }
        tmp = day_dir / (REVIEW_FILENAME + ".tmp")
        tmp.write_text(json.dumps(doc, indent=2, default=str), encoding="utf-8")
        tmp.replace(path)  # atomic on the same filesystem
    return jsonify({"ok": True, "updated": doc["updated"]})


@app.get("/")
def index():
    return Response(PAGE, mimetype="text/html")


# -- page ----------------------------------------------------------------------

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Meal Review</title>
<style>
  :root {
    --bg:#f4f5f7; --surface:#ffffff; --text:#1b2130; --muted:#667085;
    --border:#e3e6ec; --shadow:0 1px 2px rgba(16,24,40,.05), 0 2px 6px rgba(16,24,40,.06);
    --intervention:#d64545; --explanation:#2f6fed; --note:#c2620c;
    --accent:#2f6fed; --start:#12805c; --danger:#b42318;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg:#0f1216; --surface:#171b22; --text:#e7e9ee; --muted:#98a1b0;
      --border:#272c36; --shadow:none;
      --intervention:#e5605f; --explanation:#5b8bf5; --note:#f4b13f;
      --accent:#5b8bf5; --start:#1f9d72; --danger:#e5605f;
    }
  }
  * { box-sizing:border-box; margin:0; -webkit-tap-highlight-color:transparent; }
  body { font-family:-apple-system, 'Segoe UI', Roboto, sans-serif; background:var(--bg);
         color:var(--text); -webkit-font-smoothing:antialiased; }
  #app { max-width:720px; margin:0 auto; padding:20px 18px 40px; }

  h1 { font-size:16px; font-weight:600; color:var(--muted); letter-spacing:.02em; margin-bottom:14px; }
  label.cap { display:block; font-size:12px; font-weight:600; color:var(--muted);
         text-transform:uppercase; letter-spacing:.06em; margin-bottom:8px; }

  .bar { display:flex; gap:8px; align-items:flex-end; flex-wrap:wrap; margin-bottom:14px; }
  .bar .field { flex:1; min-width:200px; }
  select, input, textarea { font-family:inherit; color:var(--text); background:var(--surface);
         border:1px solid var(--border); border-radius:10px; box-shadow:var(--shadow); }
  select { width:100%; padding:12px; font-size:15px; appearance:none; }
  .meta { font-size:13px; color:var(--muted); margin-bottom:18px; line-height:1.5; }

  .card { background:var(--surface); border:1px solid var(--border); border-left:5px solid var(--muted);
         border-radius:12px; box-shadow:var(--shadow); padding:14px 15px; margin-bottom:12px; }
  .card.intervention { border-left-color:var(--intervention); }
  .card.explanation { border-left-color:var(--explanation); }
  .card.note { border-left-color:var(--note); }
  .card-head { display:flex; align-items:center; gap:10px; margin-bottom:10px; }
  .card-head select { width:auto; flex:0 0 auto; padding:7px 10px; font-size:13px; font-weight:600; }
  .card-times { flex:1; font-size:12.5px; color:var(--muted); font-variant-numeric:tabular-nums; }
  .tag { font-size:11px; font-weight:600; color:var(--muted); border:1px solid var(--border);
         border-radius:20px; padding:2px 9px; }
  .del { background:none; border:none; color:var(--muted); font-size:13px; font-weight:600;
         cursor:pointer; padding:6px 8px; }
  .del:hover { color:var(--danger); }
  .card textarea { width:100%; min-height:64px; resize:vertical; font-size:15px; line-height:1.45;
         padding:10px 12px; }
  textarea:focus, select:focus { outline:2px solid var(--accent); outline-offset:0; border-color:transparent; }

  #reflections { width:100%; min-height:140px; resize:vertical; font-size:15px; line-height:1.5;
         padding:12px 14px; margin-bottom:16px; }

  .btn { border:none; border-radius:12px; font-family:inherit; font-size:15px; font-weight:600;
         cursor:pointer; padding:13px 18px; }
  .btn.ghost { background:var(--surface); color:var(--text); border:1px solid var(--border);
         box-shadow:var(--shadow); }
  .btn.primary { background:var(--accent); color:#fff; width:100%; padding:16px; font-size:16px; }
  .btn:disabled { opacity:.45; cursor:default; }
  .row-add { margin:4px 0 22px; }
  .section-title { font-size:14px; font-weight:700; margin:26px 0 12px; }

  #status { text-align:center; font-size:13px; color:var(--muted); margin-top:12px; min-height:18px; }
  #banner { display:none; background:var(--danger); color:#fff; border-radius:12px;
         padding:12px 14px; margin-top:16px; font-size:14px; text-align:center; }
  #banner.show { display:block; }
  .hidden { display:none; }
</style>
</head>
<body>
<main id="app">
  <h1>Meal Review</h1>

  <div class="bar">
    <div class="field">
      <label class="cap" for="session-select">Session</label>
      <select id="session-select"><option value="" disabled selected>Loading…</option></select>
    </div>
    <button class="btn ghost" id="reload">Reload</button>
  </div>
  <div class="meta" id="meta">Pick a session to review its log.</div>

  <div id="editor" class="hidden">
    <div class="section-title">Interventions, explanations &amp; notes</div>
    <div id="entries"></div>
    <div class="row-add"><button class="btn ghost" id="add-entry">+ Add entry</button></div>

    <div class="section-title">End-of-meal notes &amp; your thoughts</div>
    <textarea id="reflections" placeholder="Overall reflections, things worth noting for analysis, follow-ups…"></textarea>

    <button class="btn primary" id="save">Save review</button>
    <div id="status"></div>
  </div>

  <div id="banner">Server unreachable — your changes are NOT being saved.</div>
</main>
<script>
'use strict';
const $ = id => document.getElementById(id);
const esc = s => String(s == null ? '' : s).replace(/[&<>"]/g,
  c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const KINDS = ['intervention', 'explanation', 'note'];

let sessions = [];
let cur = null;        // {dir, user, day}
let entries = [];      // working model (incl. tombstoned deleted:true)
let meal = {start: null, end: null};

const two = n => String(n).padStart(2, '0');
function clock(epoch) {
  if (epoch == null) return '—';
  const d = new Date(epoch * 1000);
  return two(d.getHours()) + ':' + two(d.getMinutes()) + ':' + two(d.getSeconds());
}
function dur(a, b) {
  if (a == null || b == null || b < a) return '';
  let s = Math.round(b - a);
  return ' (' + Math.floor(s / 60) + 'm ' + two(s % 60) + 's)';
}

async function loadSessions() {
  try {
    const d = await (await fetch('/api/review')).json();
    sessions = d.sessions || [];
    $('banner').classList.remove('show');
    const sel = $('session-select');
    sel.innerHTML = '<option value="" disabled ' + (cur ? '' : 'selected') + '>Choose a session…</option>'
      + sessions.map(s => `<option value="${esc(s.dir)}"${cur && cur.dir === s.dir ? ' selected' : ''}>`
          + `${esc(s.label)}${s.reviewed ? ' • reviewed' : ''}</option>`).join('');
  } catch (e) { $('banner').classList.add('show'); }
}

async function loadSession(dir) {
  try {
    const d = await (await fetch('/api/review?dir=' + encodeURIComponent(dir))).json();
    if (d.error) { $('meta').textContent = d.error; return; }
    cur = d.session;
    meal = d.review.meal || {start: null, end: null};
    entries = (d.review.entries || []).map(e => Object.assign({}, e));
    $('reflections').value = d.review.reflections || '';
    $('editor').classList.remove('hidden');
    const savedTxt = d.saved
      ? `Loaded your saved review (last updated ${d.review.updated ? clock(d.review.updated.epoch) : '?'}).`
      : 'No review saved yet — seeded from the original log. The original file is left untouched.';
    $('meta').innerHTML = `<b>${esc(cur.user)} / day ${cur.day}</b> · meal `
      + `${clock(meal.start)} → ${clock(meal.end)}${dur(meal.start, meal.end)}<br>${esc(savedTxt)}`;
    renderEntries();
    $('status').textContent = '';
    $('banner').classList.remove('show');
  } catch (e) { $('banner').classList.add('show'); }
}

function renderEntries() {
  const box = $('entries');
  box.innerHTML = '';
  const live = entries.map((e, i) => [e, i]).filter(([e]) => !e.deleted);
  if (!live.length) {
    box.innerHTML = '<div class="meta">No entries yet — add one below.</div>';
  }
  for (const [e, i] of live) {
    const card = document.createElement('div');
    card.className = 'card ' + e.kind;
    card.dataset.idx = i;
    const opts = KINDS.map(k => `<option value="${k}"${k === e.kind ? ' selected' : ''}>`
        + k.charAt(0).toUpperCase() + k.slice(1) + '</option>').join('');
    const when = e.origin === 'original'
      ? `${clock(e.start)} → ${clock(e.end)}${dur(e.start, e.end)}`
      : 'added in review';
    card.innerHTML =
      `<div class="card-head">`
      + `<select class="kind">${opts}</select>`
      + `<span class="card-times">${esc(when)} <span class="tag">${e.origin}</span></span>`
      + `<button class="del" type="button">Remove</button>`
      + `</div>`
      + `<textarea class="note" placeholder="Note…">${esc(e.note)}</textarea>`;
    card.querySelector('.kind').onchange = ev => {
      entries[i].kind = ev.target.value; card.className = 'card ' + ev.target.value;
    };
    card.querySelector('.note').oninput = ev => { entries[i].note = ev.target.value; };
    card.querySelector('.del').onclick = () => { entries[i].deleted = true; renderEntries(); };
    box.appendChild(card);
  }
}

function addEntry() {
  entries.push({id: null, kind: 'note', start: null, end: null, note: '',
                origin: 'added', deleted: false});
  renderEntries();
}

async function save() {
  if (!cur) return;
  $('save').disabled = true;
  $('status').textContent = 'Saving…';
  try {
    const r = await (await fetch('/api/review', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({dir: cur.dir, meal, entries,
                            reflections: $('reflections').value}),
    })).json();
    if (r.error) { $('status').textContent = 'Error: ' + r.error; }
    else {
      $('status').textContent = 'Saved to researcher_review.json at ' + clock(r.updated.epoch)
        + ' — original log untouched.';
      loadSessions();  // refresh the "reviewed" marker
    }
  } catch (e) {
    $('banner').classList.add('show'); $('status').textContent = 'Save failed — server unreachable.';
  } finally { $('save').disabled = false; }
}

$('session-select').onchange = e => loadSession(e.target.value);
$('reload').onclick = () => { loadSessions(); if (cur) loadSession(cur.dir); };
$('add-entry').onclick = addEntry;
$('save').onclick = save;
loadSessions();
</script>
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--port", type=int, default=PORT, help=f"port (default {PORT})")
    args = parser.parse_args()

    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    print(f"[review_meal] {datetime.now().isoformat(timespec='seconds')} "
          f"serving on http://0.0.0.0:{args.port}")
    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
