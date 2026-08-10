"""
Transport controls over a finished recording. Shared by the live demos.

Not a demo itself -- the leading underscore says so. It holds no simulation
knowledge at all: give it a figure, a time vector, and a function that draws
frame i, and it provides play, pause, step, seek and speed over them.

Playback is a timer stepping an index, not a FuncAnimation, because the index
has to be settable from a slider and a keypress as well as by the clock.
Scrubbing an animation that owns its own frame counter means fighting it;
owning the index outright is simpler and makes pause, step and seek the same
operation with different arguments.
"""

from __future__ import annotations

import time

CONTROLS_HELP = (
    "  controls: space play/pause, left/right step, home/end,"
    "\n            up/down speed, or drag the time bar"
)


class Player:
    """Media-player transport over a recording sampled every `dt` seconds."""

    def __init__(self, fig, log, update, speed, fps, dt):
        from matplotlib.widgets import Button, Slider

        self.fig, self.log, self.update = fig, log, update
        self.dt = float(dt)
        self.n = len(log["t"])
        self.speed = float(speed)
        self.fps = fps
        self.idx = 0
        self.playing = True
        self._last_tick = time.monotonic()
        self._measured_fps = float(fps)

        bar = dict(color="0.92", hovercolor="0.82")
        self.ax_slider = fig.add_axes([0.06, 0.065, 0.91, 0.022])
        self.slider = Slider(
            self.ax_slider, "", 0.0, float(log["t"][-1]),
            valinit=0.0, color="C0", track_color="0.88",
        )
        self.slider.valtext.set_visible(False)
        self.slider.on_changed(self._seek)

        def button(x, w, label, cb):
            b = Button(fig.add_axes([x, 0.018, w, 0.032]), label, **bar)
            b.label.set_fontsize(9)
            b.on_clicked(cb)
            return b

        self.b_start = button(0.06, 0.05, "|<", lambda _: self._goto(0))
        self.b_back = button(0.115, 0.05, "<<", lambda _: self._nudge(-1))
        self.b_play = button(0.17, 0.07, "pause", self._toggle)
        self.b_fwd = button(0.245, 0.05, ">>", lambda _: self._nudge(+1))
        self.b_end = button(0.30, 0.05, ">|", lambda _: self._goto(self.n - 1))
        self.b_slow = button(0.365, 0.05, "- speed", lambda _: self._scale(0.5))
        self.b_fast = button(0.42, 0.05, "+ speed", lambda _: self._scale(2.0))

        self.clock = fig.text(
            0.50, 0.033, "", family="monospace", fontsize=10, va="center",
        )
        fig.text(
            0.72, 0.033,
            "space play/pause   left/right step   home/end   up/down speed",
            fontsize=8, color="0.4", va="center",
        )
        fig.canvas.mpl_connect("key_press_event", self._key)

        self.timer = fig.canvas.new_timer(interval=int(1000.0 / fps))
        self.timer.add_callback(self._tick)
        self.timer.start()
        self._render()

    # -- state changes ---------------------------------------------------

    def _goto(self, i, from_slider=False):
        self.idx = int(min(max(i, 0), self.n - 1))
        if not from_slider:
            # Setting the slider re-enters _seek; guard so the two cannot
            # chase each other.
            self._syncing = True
            self.slider.set_val(self.log["t"][self.idx])
            self._syncing = False
        self._render()

    def _seek(self, value):
        if getattr(self, "_syncing", False):
            return
        self.playing = False
        self.b_play.label.set_text("play")
        self._goto(int(round(value / self.dt)), from_slider=True)

    @property
    def _step(self) -> int:
        """Indices in one nominal display frame at the current speed."""
        return max(1, int(round(self.speed / (self.fps * self.dt))))

    def _nudge(self, direction):
        self.playing = False
        self.b_play.label.set_text("play")
        self._goto(self.idx + direction * self._step)

    def _toggle(self, _event=None):
        # Restart from the beginning if play is pressed at the very end,
        # rather than appearing to do nothing.
        if not self.playing and self.idx >= self.n - 1:
            self.idx = 0
        self.playing = not self.playing
        self.b_play.label.set_text("pause" if self.playing else "play")
        self.fig.canvas.draw_idle()

    def _scale(self, factor):
        self.speed = min(max(self.speed * factor, 0.25), 256.0)
        self._render()

    def _key(self, event):
        if event.key == " ":
            self._toggle()
        elif event.key == "right":
            self._nudge(+1)
        elif event.key == "left":
            self._nudge(-1)
        elif event.key == "home":
            self._goto(0)
        elif event.key == "end":
            self._goto(self.n - 1)
        elif event.key == "up":
            self._scale(2.0)
        elif event.key == "down":
            self._scale(0.5)

    # -- the loop --------------------------------------------------------

    def _tick(self):
        """Advance by wall-clock time, not by a fixed number of steps.

        Stepping a fixed stride per frame silently ties playback speed to
        render cost: these figures take about 80 ms to draw, so a timer asking
        for 25 fps gets 12.5, and a nominal 4x played at 2x while the readout
        insisted it was 4x. Deriving the advance from elapsed wall time makes
        the speed honest -- when rendering cannot keep up, frames are dropped
        rather than time dilated, which is what a media player does.
        """
        now = time.monotonic()
        elapsed = now - self._last_tick
        self._last_tick = now
        if elapsed > 0.0:
            # Smoothed, only for the readout, so a reader can see when the
            # display is struggling rather than wonder why it looks choppy.
            self._measured_fps += 0.2 * (1.0 / elapsed - self._measured_fps)

        if not self.playing:
            return
        if self.idx >= self.n - 1:
            self.playing = False
            self.b_play.label.set_text("play")
            return

        # Clamp the jump so a stall or a long pause does not skip the mission.
        elapsed = min(elapsed, 0.25)
        self._goto(self.idx + max(1, int(round(self.speed * elapsed / self.dt))))

    def _render(self):
        self.update(self.idx)
        now, total = self.log["t"][self.idx], self.log["t"][-1]
        self.clock.set_text(
            f"{now:6.1f} / {total:.0f} s   -{total - now:5.1f} s"
            f"   x{self.speed:g}   {self._measured_fps:4.1f} fps"
        )
        self.fig.canvas.draw_idle()
