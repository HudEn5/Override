import test from 'node:test';
import assert from 'node:assert/strict';
import { spawn, spawnSync } from 'node:child_process';
import { once } from 'node:events';
import { mkdtempSync, readFileSync, writeFileSync, unlinkSync, rmSync, realpathSync, mkdirSync } from 'node:fs';
import { basename, dirname, join, resolve, sep } from 'node:path';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';

const script = resolve(dirname(fileURLToPath(import.meta.url)), '../scripts/sync-github.ps1');
function git(directory, ...args) {
  const result = spawnSync('git', ['-C', directory, '-c', 'user.name=Sync Test',
    '-c', 'user.email=sync-test@example.invalid', ...args], { encoding: 'utf8', windowsHide: true });
  assert.equal(result.status, 0, result.stderr || result.stdout);
  return result.stdout.trim();
}

test('Windows GitHub sync preserves work and only fast-forwards', { skip: process.platform !== 'win32' }, async t => {
  const root = mkdtempSync(join(tmpdir(), 'vex-device-sync-'));
  t.after(() => {
    const target = realpathSync(root);
    assert.ok(target.toLowerCase().startsWith((realpathSync(tmpdir()) + sep).toLowerCase()));
    assert.ok(basename(target).startsWith('vex-device-sync-'));
    rmSync(target, { recursive: true, force: true });
  });
  const remote = join(root, 'origin.git');
  const writer = join(root, 'writer laptop');
  git(root, 'init', '--bare', '--initial-branch=main', remote);
  git(root, 'clone', remote, writer);
  writeFileSync(join(writer, 'program.txt'), 'v1\n');
  writeFileSync(join(writer, '.gitattributes'), '*.txt text eol=lf\n');
  writeFileSync(join(writer, '.gitignore'), 'generated.txt\n.vex-device.json\n');
  git(writer, 'add', '.');
  git(writer, 'commit', '-m', 'Initial program');
  git(writer, 'push', 'origin', 'main');
  function clone(name) {
    const directory = join(root, name);
    git(root, 'clone', remote, directory);
    return directory;
  }
  function sync(directory, ...extra) {
    const result = spawnSync('powershell.exe', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', script,
      '-RepositoryPath', directory, '-ExpectedRemoteUrl', remote, ...extra],
    { encoding: 'utf8', windowsHide: true, timeout: 60_000 });
    assert.ifError(result.error);
    return { status: result.status, output: result.stdout + result.stderr };
  }
  const expect = (directory, code, text, ...extra) => {
    const result = sync(directory, ...extra);
    assert.equal(result.status, code, result.output);
    assert.match(result.output, text);
  };

  await t.test('receives a laptop commit in a path containing spaces', () => {
    const directory = clone('desktop clone');
    writeFileSync(join(writer, 'program.txt'), 'v2\n');
    git(writer, 'add', 'program.txt');
    git(writer, 'commit', '-m', 'Laptop tuning');
    git(writer, 'push', 'origin', 'main');
    expect(directory, 0, /\[updated\]/);
    assert.equal(readFileSync(join(directory, 'program.txt'), 'utf8'), 'v2\n');
    expect(directory, 0, /\[current\]/);
  });

  await t.test('pauses for unstaged, staged and untracked work', () => {
    const directory = clone('dirty');
    const file = join(directory, 'program.txt');
    const head = git(directory, 'rev-parse', 'HEAD');
    writeFileSync(file, 'local tuning\n');
    expect(directory, 2, /\[paused\]/);
    git(directory, 'add', 'program.txt');
    expect(directory, 2, /\[paused\]/);
    assert.equal(readFileSync(file, 'utf8'), 'local tuning\n');
    writeFileSync(file, 'v2\n');
    git(directory, 'add', 'program.txt');
    writeFileSync(join(directory, 'lab-note.txt'), 'keep this');
    expect(directory, 2, /\[paused\]/);
    assert.equal(git(directory, 'rev-parse', 'HEAD'), head);
    unlinkSync(join(directory, 'lab-note.txt'));
    expect(directory, 0, /\[current\]/);
  });

  await t.test('preserves a feature branch and detached HEAD', () => {
    const directory = clone('branches');
    git(directory, 'switch', '-c', 'experiment');
    expect(directory, 2, /\[paused\]/);
    assert.equal(git(directory, 'branch', '--show-current'), 'experiment');
    git(directory, 'switch', '--detach');
    expect(directory, 2, /\[paused\]/);
  });

  await t.test('does not push local commits or merge diverged history', () => {
    const directory = clone('commits');
    writeFileSync(join(directory, 'desktop.txt'), 'local commit');
    git(directory, 'add', '.');
    git(directory, 'commit', '-m', 'Desktop work');
    const head = git(directory, 'rev-parse', 'HEAD');
    const remoteHead = git(writer, 'ls-remote', 'origin', 'refs/heads/main');
    expect(directory, 2, /\[paused\]/);
    assert.equal(git(writer, 'ls-remote', 'origin', 'refs/heads/main'), remoteHead);
    writeFileSync(join(writer, 'program.txt'), 'v3\n');
    git(writer, 'add', 'program.txt');
    git(writer, 'commit', '-m', 'New laptop tuning');
    git(writer, 'push', 'origin', 'main');
    expect(directory, 2, /\[paused\]/);
    assert.equal(git(directory, 'rev-parse', 'HEAD'), head);
  });

  await t.test('refuses to overwrite an ignored local file', () => {
    const directory = clone('ignored');
    writeFileSync(join(directory, 'generated.txt'), 'local data');
    writeFileSync(join(writer, 'generated.txt'), 'remote data');
    git(writer, 'add', '--force', 'generated.txt');
    git(writer, 'commit', '-m', 'New tracked file');
    git(writer, 'push', 'origin', 'main');
    expect(directory, 1, /overwritten/);
    assert.equal(readFileSync(join(directory, 'generated.txt'), 'utf8'), 'local data');
  });

  await t.test('rejects the wrong origin and a nested non-root directory', () => {
    const directory = clone('wrong-origin');
    git(directory, 'remote', 'set-url', 'origin', join(root, 'other.git'));
    expect(directory, 1, /origin does not match/);
    const child = join(directory, 'nested');
    mkdirSync(child);
    expect(child, 1, /not the repository root/);
  });

  await t.test('automatic mode stays off on an unconfigured device or laptop', () => {
    const directory = clone('laptop');
    expect(directory, 0, /\[disabled\]/, '-Auto');
    writeFileSync(join(directory, '.vex-device.json'), JSON.stringify({ role: 'Laptop', autoPull: true }));
    expect(directory, 0, /\[disabled\]/, '-Auto');
  });

  await t.test('only one sync worker can run for the same checkout', async () => {
    const directory = clone('single worker');
    const child = spawn('powershell.exe', ['-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', script,
      '-RepositoryPath', directory, '-ExpectedRemoteUrl', remote, '-Watch'], { windowsHide: true });
    const closed = once(child, 'close');
    try {
      await new Promise((resolveReady, reject) => {
        const timer = setTimeout(() => reject(new Error('Watcher did not start')), 15_000);
        let output = '';
        child.stdout.on('data', data => {
          output += data;
          if (output.includes('[current]')) { clearTimeout(timer); resolveReady(); }
          if (output.includes('[error]')) { clearTimeout(timer); reject(new Error(output)); }
        });
        child.once('error', error => { clearTimeout(timer); reject(error); });
      });
      expect(directory, 0, /\[busy\]/);
    } finally {
      child.kill();
      await closed;
    }
  });
});
