using System.Diagnostics;

namespace GagameDemo;

/// One child process at a time, its output streamed line by line to the UI thread. Both forms use
/// it: the install form for harvesters and the prune listing, the main form for the engine.
public sealed class ProcessRunner : IDisposable
{
    readonly SynchronizationContext _ui;
    readonly Action<string> _log;
    readonly Action<bool> _running;
    Process? _proc;

    public ProcessRunner(SynchronizationContext ui, Action<string> log, Action<bool> running)
    {
        _ui = ui;
        _log = log;
        _running = running;
    }

    public bool IsRunning => _proc is { HasExited: false };
    public int? Pid => IsRunning ? _proc!.Id : null;

    /// Starts `psi`; `title` is what the log calls it. Returns false when something is already running.
    public bool Start(ProcessStartInfo psi, string title)
    {
        if (IsRunning) return false;
        var p = new Process { StartInfo = psi, EnableRaisingEvents = true };
        p.OutputDataReceived += (_, e) => { if (e.Data is not null) Post(e.Data); };
        p.ErrorDataReceived += (_, e) => { if (e.Data is not null) Post(e.Data); };
        p.Exited += (_, _) =>
        {
            var code = p.ExitCode;
            _ui.Post(_ =>
            {
                _log($"[demo] {title} exited with code {code}");
                _running(false);
            }, null);
        };
        try
        {
            Post($"[demo] {title}: {psi.FileName} {string.Join(' ', psi.ArgumentList)}");
            p.Start();
            p.BeginOutputReadLine();
            p.BeginErrorReadLine();
        }
        catch (Exception ex)
        {
            Post($"[demo] could not start {title}: {ex.Message}");
            p.Dispose();
            return false;
        }
        _proc = p;
        _running(true);
        return true;
    }

    public void Stop()
    {
        if (!IsRunning) return;
        try { _proc!.Kill(entireProcessTree: true); }
        catch (Exception ex) { Post($"[demo] stop: {ex.Message}"); }
    }

    void Post(string line) => _ui.Post(_ => _log(line), null);

    public void Dispose()
    {
        Stop();
        _proc?.Dispose();
        _proc = null;
    }
}
