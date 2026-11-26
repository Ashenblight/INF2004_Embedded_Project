# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the LED blinking, the motor turning, the robot reaching
the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

| Date       | Subsystem             | Test ID / demo                                                  | Commit    | Student ID | Link                                                                                 | Notes                                     |
| ---------- | --------------------- | --------------------------------------------------------------- | --------- | ---------- | ------------------------------------------------------------------------------------ | ----------------------------------------- |
| 2026-11-26 | integration subsystem | testing right turn at cross junction                            |           |            | <https://drive.google.com/file/d/1hqQV869TZ73VOC27WoFm2LwM9rFyb6K2/view?usp=sharing> | `cross-junction_right-turn_test.mp4`      |
| 2026-11-26 | integration subsystem | testing line following right turn 1                             |           |            | <https://drive.google.com/file/d/1CgGwQCU4sWL5hDEDVDBAwW4xUfu1i8Cb/view?usp=sharing> | `line-following_right-turn_test1.mp4`     |
| 2026-11-26 | integration subsystem | testing line following right turn 1                             |           |            | <https://drive.google.com/file/d/1IuFQX-iNrDiwu_b_5kUaIghGWPRau0oe/view?usp=sharing> | `line-following_right-turn_test2.mp4`     |
| 2026-11-26 | integration subsystem | testing line following + right turn at cross junction in a loop |           |            | <https://drive.google.com/file/d/1doZKU5Ft2fuKEKC7J8VFB863nJzPuNKm/view?usp=sharing> | `cross-junction_right-turn_test_loop.mp4` |
| 2026-11-26 |                       | demo try 2                                                      |           |            | <https://drive.google.com/file/d/1UjdCFTdDrz-hmAo4MeG_EYtY9lVsCrB1/view?usp=sharing> | `demo_run2.mp4`                           |
