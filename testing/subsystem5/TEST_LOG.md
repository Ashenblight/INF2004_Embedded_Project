# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Initial task-context check (retired) | | auto | Earlier test image | Located a running test task |
| S5-U02 | | | manual | 1. … 2. … | |
| S5-U03 | Axle advance after braking travel | Junction geometry | auto | Supply one mm of forward travel at both wheels | One mm less travel remains |
| S5-U04 | Axle advance after reverse travel | Junction geometry | auto | Supply one mm of reverse travel at both wheels | Full axle-to-IR distance remains |
| S5-U05 | Signed U-turn target | Junction geometry | auto | Split the configured command into two pivots | The two commands sum to the configured target |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, oscilloscope, or meter).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
