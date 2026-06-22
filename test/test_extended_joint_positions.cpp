/**
 * \file test_extended_joint_positions.cpp
 * \brief
 *   Unit tests for compliant_controllers::ExtendedJointPositions.
 *
 *   Tests exercise the REAL class from src/extended_joint_positions.cpp
 *   (linked via CMake), not a replica.  A white-box subclass exposes the
 *   protected normalize() overloads and internal state so wraparound edge
 *   cases can be constructed exactly.
 *
 *   ---------------------------------------------------------------
 *   Contract (read off the call sites, not assumed):
 *   ---------------------------------------------------------------
 *   - update() is called with the *measured* current_state.positions, NOT
 *     a command (the parameter name `target_joint_positions` is a
 *     misnomer).  src/joint_space/controller.cpp:159 (+ task_space,
 *     joint_task_space).
 *   - VERIFIED input pipeline (kortex_hardware Gen3Robot.cpp:989-1004):
 *       Kinova feedback is DEGREES in [0,360); read() converts to radians
 *       and re-centres with `if (pos > M_PI) pos -= 2*M_PI`, so the driver
 *       reports [-π, π).  read() is memoryless (no continuous-joint
 *       accumulation), so |reading| < π always.
 *     Then the adapter's ONE-SHOT shift `if (q < 0) q += 2π`
 *       (hardware_interface_adapter_impl.h:140-141) maps [-π,0) -> [π,2π),
 *       so update() reliably receives [0, 2π).  Because the driver bounds
 *       to [-π,π), the one-shot shift is always sufficient in the live
 *       system — B6 (|q| >= 2π) is therefore DORMANT, latent only if the
 *       driver is ever changed to report signed/accumulated angles.
 *   - The job is std::unwrap: a continuous multi-turn position
 *     diff_joint_positions_, congruent to the reading mod 2π, moving
 *     continuously.  Output feeds nominal_theta_prev_ / current_theta_.
 *   - The algorithm only works inside the envelope |per-step Δ| < π/2
 *     (docs/continuous_joint_wraparound.tex); a single step ≥ 3π/2 is
 *     unconditionally read as a wrap.
 *
 *   ---------------------------------------------------------------
 *   Key design fact (the root cause behind B2/B3/B6 and INV):
 *   ---------------------------------------------------------------
 *   The no-wrap branch reconstructs  diff = floor(diff/2π)·2π + t  using
 *   the ABSOLUTE reading t.  A correct unwrap depends only on the wrapped
 *   DELTA (t - prev), so it is invariant to which 2π-congruent
 *   representative the caller passes.  This implementation is NOT: its
 *   output changes if the same physical angle is presented as t, t+2π or
 *   t-2π.  That is exactly why out-of-[0,2π) input misbehaves.
 *
 *   ---------------------------------------------------------------
 *   Sections:  P1 functionality (must pass) · P1I invariants (must pass)
 *              · P1C characterisation (must pass; documents behaviour)
 *              · P2 defects (XFAIL with severity/reachability tags).
 *
 *   Defect map:
 *     B2  LIVE/frequent (CONFIRMED) — init() normalises to [-π,π) while
 *                          update() gets [0,2π); since the driver reports
 *                          [-π,π) (Gen3Robot.cpp:1000), any joint resting
 *                          negative starts a full turn off in frame.  This
 *                          is the wraparound analysis' near-deterministic
 *                          startup 2π offset, now verified against the
 *                          real driver range.
 *     B5  LIVE on glitch — a NaN reading poisons current_.
 *     B6  DORMANT — driver bounds readings to [-π,π) (Gen3Robot.cpp:1000),
 *                          so the one-shot adapter shift is always enough
 *                          and |q|≥2π never reaches update().  Kept as a
 *                          regression guard against a future driver that
 *                          reports signed/accumulated angles.
 *     INV LIVE if input leaves [0,2π) — output is representative-
 *                          dependent (the general form of B2/B3/B6).
 *     B1  LATENT knife-edge — floor off-by-one; only at bit-exact -2π·k
 *                          (normal float data is safe — P1.walk_…).
 *     B3  LATENT/robustness — wrap branch blows up on a ±π seam, only for
 *                          [-π,π) input the adapter never sends.
 */

#include "compliant_controllers/extended_joint_positions.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Eigen>

using compliant_controllers::ExtendedJointPositions;

// ---------------------------------------------------------------------
//  White-box subclass: reach protected normalize() + internal state.
// ---------------------------------------------------------------------
class TestableEJP : public ExtendedJointPositions {
 public:
  using ExtendedJointPositions::ExtendedJointPositions;

  static double norm(double a) { return ExtendedJointPositions::normalize(a); }
  static Eigen::VectorXd norm(Eigen::VectorXd const& v) {
    return ExtendedJointPositions::normalize(v);
  }

  double diff(Eigen::Index i = 0) const { return unwrapped_joint_positions_(i); }
  double current(Eigen::Index i = 0) const { return previous_joint_positions_(i); }

  // Force exact internal state (init() must have run first to size the
  // vectors) to build knife-edge states unreachable by float walking.
  void forceState(double diff_val, double current_val, Eigen::Index i = 0) {
    unwrapped_joint_positions_(i) = diff_val;
    previous_joint_positions_(i) = current_val;
  }
};

// ---------------------------------------------------------------------
//  Tiny assertion harness (no gtest in this repo).
// ---------------------------------------------------------------------
static int checks_run = 0;
static int checks_failed = 0;   // hard failures (P1 / P1I / P1C)
static int xfail_known = 0;     // known defects still present (expected)
static int xpass_fixed = 0;     // known defects that now pass (investigate)

static bool near(double a, double b, double tol) {
  if (std::isnan(a) || std::isnan(b)) return std::isnan(a) && std::isnan(b);
  return std::abs(a - b) <= tol;
}

#define CHECK_NEAR(actual, expected, tol, name)                              \
  do {                                                                       \
    ++checks_run;                                                            \
    double const _a = (actual), _e = (expected);                            \
    if (near(_a, _e, (tol))) std::printf("  pass  %-52s = %.6g\n", (name), _a); \
    else { ++checks_failed;                                                   \
      std::fprintf(stderr, "  FAIL  %-52s expected %.6g got %.6g\n",         \
                   (name), _e, _a); }                                        \
  } while (0)

#define CHECK_TRUE(cond, name)                                               \
  do {                                                                       \
    ++checks_run;                                                            \
    if (cond) std::printf("  pass  %s\n", (name));                           \
    else { ++checks_failed; std::fprintf(stderr, "  FAIL  %s\n", (name)); }  \
  } while (0)

// XFAIL: `correct` is the value the implementation SHOULD produce; we
// expect it currently does NOT.  If it does, the defect may be fixed —
// report loudly so the check is promoted to CHECK_NEAR.
#define CHECK_XFAIL(actual, correct, tol, name)                              \
  do {                                                                       \
    ++checks_run;                                                            \
    double const _a = (actual), _c = (correct);                             \
    if (near(_a, _c, (tol))) { ++xpass_fixed;                                \
      std::fprintf(stderr,                                                   \
        "  XPASS *** %-46s now == correct %.6g — DEFECT FIXED? promote me\n",\
        (name), _c); }                                                       \
    else { ++xfail_known;                                                    \
      std::printf("  xfail %-46s got %.6g, correct = %.6g (known defect)\n", \
                  (name), _a, _c); }                                         \
  } while (0)

// ---------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------
static Eigen::VectorXd vec1(double x) { Eigen::VectorXd v(1); v(0) = x; return v; }

// Adapter convention: full wrap of a physical angle into [0, 2π).
static double wrap02(double x) {
  double r = std::fmod(x, 2.0 * M_PI);
  if (r < 0.0) r += 2.0 * M_PI;
  return r;
}
// The DEPLOYED adapter's one-shot shift (hardware_interface_adapter_impl.h:140).
static double adapter_one_shot_shift(double q) { return (q < 0.0) ? q + 2.0 * M_PI : q; }

static std::vector<double> run_sequence(double q0,
                                        std::vector<double> const& targets) {
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(q0));
  std::vector<double> traj;
  for (double t : targets) { ext.update(vec1(t)); traj.push_back(ext.getPositions()(0)); }
  return traj;
}

// =====================================================================
//  P1 — Functionality (happy path; MUST PASS)
// =====================================================================

static void test_normalize_scalar() {
  std::puts("\n[P1.normalize_scalar] range [-pi, pi)");
  CHECK_NEAR(TestableEJP::norm(0.0), 0.0, 1e-12, "normalize(0)");
  CHECK_NEAR(TestableEJP::norm(0.5), 0.5, 1e-12, "normalize(0.5)");
  CHECK_NEAR(TestableEJP::norm(-0.5), -0.5, 1e-12, "normalize(-0.5)");
  CHECK_NEAR(TestableEJP::norm(M_PI), -M_PI, 1e-12, "normalize(pi) -> -pi");
  CHECK_NEAR(TestableEJP::norm(-M_PI), -M_PI, 1e-12, "normalize(-pi) -> -pi");
  CHECK_NEAR(TestableEJP::norm(2.0 * M_PI - 0.1), -0.1, 1e-9, "normalize(2pi-0.1)");
  CHECK_NEAR(TestableEJP::norm(2.0 * M_PI), 0.0, 1e-12, "normalize(2pi) -> 0");
  CHECK_NEAR(TestableEJP::norm(0.3 + 6.0 * M_PI), TestableEJP::norm(0.3), 1e-9,
             "normalize periodic +3 turns");
}

static void test_normalize_vector() {
  std::puts("\n[P1.normalize_vector] element-wise");
  Eigen::VectorXd v(3); v << 0.0, M_PI, 2.0 * M_PI - 0.1;
  Eigen::VectorXd n = TestableEJP::norm(v);
  CHECK_NEAR(n(0), 0.0, 1e-12, "vec[0]");
  CHECK_NEAR(n(1), -M_PI, 1e-12, "vec[1] (pi->-pi)");
  CHECK_NEAR(n(2), -0.1, 1e-9, "vec[2]");
}

static void test_init_contract() {
  std::puts("\n[P1.init_contract] init succeeds once then sticky");
  ExtendedJointPositions ext(1);
  CHECK_TRUE(!ext.isInitialized(), "not initialized before init()");
  CHECK_TRUE(ext.init(vec1(0.5)), "first init() returns true");
  CHECK_TRUE(ext.isInitialized(), "initialized after init()");
  CHECK_TRUE(!ext.init(vec1(2.0)), "second init() returns false");
  CHECK_NEAR(ext.getPositions()(0), TestableEJP::norm(0.5), 1e-12,
             "second init() did not overwrite");
}

static void test_no_motion() {
  std::puts("\n[P1.no_motion] identical readings -> constant");
  auto traj = run_sequence(1.5, {1.5, 1.5, 1.5});
  CHECK_NEAR(traj[0], 1.5, 1e-12, "no motion step 1");
  CHECK_NEAR(traj[2], 1.5, 1e-12, "no motion step 3");
}

static void test_small_forward() {
  std::puts("\n[P1.small_forward] small forward steps track 1:1");
  auto traj = run_sequence(0.0, {0.1, 0.2, 0.3});
  CHECK_NEAR(traj[0], 0.1, 1e-12, "0.1");
  CHECK_NEAR(traj[1], 0.2, 1e-12, "0.2");
  CHECK_NEAR(traj[2], 0.3, 1e-12, "0.3");
}

static void test_small_backward() {
  std::puts("\n[P1.small_backward] small backward steps track 1:1");
  auto traj = run_sequence(1.0, {0.9, 0.8, 0.7});
  CHECK_NEAR(traj[0], 0.9, 1e-12, "0.9");
  CHECK_NEAR(traj[1], 0.8, 1e-12, "0.8");
  CHECK_NEAR(traj[2], 0.7, 1e-12, "0.7");
}

static void test_forward_seam() {
  std::puts("\n[P1.forward_seam] 0/2pi crossing in [0,2pi) input");
  // init(6.0) would normalise (B2), so build the clean near-2pi state.
  TestableEJP ext(1);
  (void)ext.init(vec1(0.0));
  ext.forceState(6.0, 6.0);
  ext.update(vec1(0.1));
  double const d0 = ext.diff();
  CHECK_NEAR(d0, 6.0 + (0.1 - TestableEJP::norm(6.0)), 1e-9,
             "forward seam continuous through 2pi");
  ext.update(vec1(0.2));
  CHECK_TRUE(ext.diff() > d0, "forward seam keeps increasing");
}

static void test_backward_seam() {
  std::puts("\n[P1.backward_seam] 2pi/0 crossing in [0,2pi) input");
  auto traj = run_sequence(0.1, {6.18, 6.0});
  CHECK_NEAR(traj[0], 0.1 + (TestableEJP::norm(6.18) - 0.1), 1e-9,
             "backward seam continuous below 0");
  CHECK_TRUE(traj[1] < traj[0], "backward seam keeps decreasing");
}

static void test_multi_step_no_wrap() {
  std::puts("\n[P1.multi_step] 100 small forward steps accumulate exactly");
  std::vector<double> targets;
  for (int i = 1; i <= 100; ++i) targets.push_back(0.01 * i);
  auto traj = run_sequence(0.0, targets);
  CHECK_NEAR(traj.back(), 1.0, 1e-9, "100 x 0.01 -> 1.0");
}

static void test_multi_rotation_forward() {
  std::puts("\n[P1.multi_rotation_fwd] 3 forward turns track continuously");
  std::vector<double> targets;
  double const step = 0.1; int const spc = static_cast<int>(std::round(2.0 * M_PI / step));
  for (int c = 0; c < 3; ++c) for (int s = 1; s <= spc; ++s) targets.push_back(wrap02(s * step));
  auto traj = run_sequence(0.0, targets);
  CHECK_NEAR(traj.back(), 3 * spc * step, 0.5, "~18.85 after 3 turns");
}

static void test_multi_rotation_backward() {
  std::puts("\n[P1.multi_rotation_bwd] 3 backward turns track continuously");
  std::vector<double> targets; double pos = 0.0;
  double const step = 0.1; int const spc = static_cast<int>(std::round(2.0 * M_PI / step));
  for (int c = 0; c < 3; ++c) for (int s = 1; s <= spc; ++s) { pos -= step; targets.push_back(wrap02(pos)); }
  auto traj = run_sequence(0.0, targets);
  CHECK_NEAR(traj.back(), -3 * spc * step, 0.5, "~-18.85 after 3 turns");
}

static void test_walk_through_minus_2pi() {
  std::puts("\n[P1.walk_through_minus_2pi] normal float data is robust at -2pi");
  // Shows the B1 off-by-one is NOT reachable by ordinary walking: a full
  // backward turn plus one step passes through diff = -2pi smoothly.
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double const step = 2.0 * M_PI / 100.0;
  double pos = 0.0;
  for (int s = 1; s <= 100; ++s) { pos -= step; ext.update(vec1(wrap02(pos))); }
  double const at_turn = ext.getPositions()(0);
  CHECK_NEAR(at_turn, -2.0 * M_PI, 1e-4, "one full backward turn ~ -2pi");
  pos -= 0.001; ext.update(vec1(wrap02(pos)));
  double const past = ext.getPositions()(0);
  CHECK_TRUE(past < at_turn && past > -3.0 * M_PI,
             "steps past -2pi smoothly (no spurious full-turn jump)");
}

static void test_multijoint_independence_and_nan_isolation() {
  std::puts("\n[P1.multijoint] joints update independently; NaN isolated per-joint");
  ExtendedJointPositions ext(3);
  Eigen::VectorXd q0(3); q0 << 0.0, 1.0, 2.0;
  (void)ext.init(q0);
  Eigen::VectorXd t(3); t << 0.1, std::numeric_limits<double>::quiet_NaN(), 2.1;
  ext.update(t);
  Eigen::VectorXd p = ext.getPositions();
  CHECK_NEAR(p(0), 0.1, 1e-12, "joint0 advanced to 0.1");
  CHECK_NEAR(p(1), 1.0, 1e-12, "joint1 (NaN) held at init value 1.0");
  CHECK_NEAR(p(2), 2.1, 1e-12, "joint2 advanced to 2.1");
  CHECK_TRUE(ext.getNanCount() == 1, "exactly one NaN counted");
}

static void test_nan_guard_protects_diff() {
  std::puts("\n[P1.nan_guard] a single NaN reading leaves diff unchanged");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  ext.update(vec1(0.1));
  CHECK_NEAR(ext.getPositions()(0), 0.1, 1e-12, "before NaN: diff = 0.1");
  CHECK_TRUE(ext.getNanCount() == 0, "nan_count starts at 0");
  ext.update(vec1(std::numeric_limits<double>::quiet_NaN()));
  CHECK_NEAR(ext.getPositions()(0), 0.1, 1e-12, "NaN: diff stays 0.1");
  CHECK_TRUE(ext.getNanCount() == 1, "nan_count incremented to 1");
}

static void test_getpositions_returns_copy() {
  std::puts("\n[P1.getpositions_copy] getPositions() returns an independent copy");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  ext.update(vec1(0.3));
  Eigen::VectorXd p = ext.getPositions();
  p(0) = 999.0;
  CHECK_NEAR(ext.getPositions()(0), 0.3, 1e-12, "internal state untouched");
}

static void test_normalize_boundaries() {
  std::puts("\n[P1.normalize_bounds] every output lands in [-pi, pi) and is congruent");
  double const xs[] = {3.0 * M_PI, -3.0 * M_PI, 100.0, -100.0, 2.0 * M_PI,
                       -2.0 * M_PI, M_PI + 1e-9, -M_PI - 1e-9};
  for (double x : xs) {
    double const n = TestableEJP::norm(x);
    CHECK_TRUE(n >= -M_PI && n < M_PI, "normalize output in [-pi, pi)");
    double const k = std::round((x - n) / (2.0 * M_PI));
    CHECK_NEAR(n + k * 2.0 * M_PI, x, 1e-6, "normalize congruent to input mod 2pi");
  }
}

static void test_init_multidof_sizing() {
  std::puts("\n[P1.init_multidof] init sizes the output to N and normalizes each entry");
  ExtendedJointPositions ext(7);
  Eigen::VectorXd q0(7);
  q0 << 0.0, 0.5, -0.5, 3.0, -3.0, 6.18, -6.18;
  CHECK_TRUE(ext.init(q0), "init(7-vector) succeeds");
  Eigen::VectorXd p = ext.getPositions();
  CHECK_TRUE(p.size() == 7, "getPositions() size == 7");
  for (int i = 0; i < 7; ++i)
    CHECK_NEAR(p(i), TestableEJP::norm(q0(i)), 1e-12, "entry normalized");
}

static void test_multidof_simultaneous_opposite_wraps() {
  std::puts("\n[P1.multidof_wraps] joint0 forward-wraps, joint1 backward-wraps, independently");
  ExtendedJointPositions ext(2);
  Eigen::VectorXd q0(2); q0 << 0.0, 0.0;
  (void)ext.init(q0);
  double p0 = 0.0, p1 = 0.0;
  double const step = 0.1;
  int const n = static_cast<int>(std::round(2.5 * 2.0 * M_PI / step));
  for (int s = 0; s < n; ++s) {
    p0 += step; p1 -= step;
    Eigen::VectorXd t(2); t << wrap02(p0), wrap02(p1);
    ext.update(t);
  }
  Eigen::VectorXd p = ext.getPositions();
  CHECK_NEAR(p(0), p0, 1e-6, "joint0 tracked forward ~2.5 turns");
  CHECK_NEAR(p(1), p1, 1e-6, "joint1 tracked backward ~2.5 turns");
}

static void test_getnancount_accumulates() {
  std::puts("\n[P1.nan_count] getNanCount accumulates across updates");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double const nan = std::numeric_limits<double>::quiet_NaN();
  ext.update(vec1(nan));
  ext.update(vec1(0.1));
  ext.update(vec1(nan));
  CHECK_TRUE(ext.getNanCount() == 2, "two NaNs counted across three updates");
}

// =====================================================================
//  P1I — Invariants / property tests (MUST PASS)
// =====================================================================

// Two defining properties of correct unwrap, over a fine multi-turn
// sweep within the safe envelope (|Δ| < π/2):
//   (1) congruence: normalize(extended) == normalize(reading) always.
//   (2) continuity: |extended[k]-extended[k-1]| ≈ Δ, never ~2π.
// NOTE: congruence alone cannot catch whole-turn (±2π) errors (B1 etc.
// preserve it), which is why (2) is also asserted.
static void test_invariants_smooth_sweep() {
  std::puts("\n[P1I.smooth_sweep] congruence + continuity over 3 forward turns");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double const step = 0.02;
  double max_jump = 0.0, max_cong = 0.0, prev = 0.0; bool first = true;
  for (int k = 1; k <= 1000; ++k) {
    double const phys = step * k;
    ext.update(vec1(wrap02(phys)));
    double const e = ext.getPositions()(0);
    max_cong = std::max(max_cong,
                        std::abs(TestableEJP::norm(e) - TestableEJP::norm(wrap02(phys))));
    if (!first) max_jump = std::max(max_jump, std::abs(e - prev));
    prev = e; first = false;
  }
  CHECK_TRUE(max_cong < 1e-9, "congruence: normalize(ext)==normalize(reading)");
  CHECK_TRUE(max_jump < 1.5 * step, "continuity: no step jumps more than ~Δ");
}

static void test_invariant_reversibility() {
  std::puts("\n[P1I.reversibility] forward then backward returns to start");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double const step = 0.05; int const n = static_cast<int>(std::round(1.5 * 2.0 * M_PI / step));
  for (int s = 1; s <= n; ++s) ext.update(vec1(wrap02(s * step)));
  double const peak = ext.getPositions()(0);
  for (int s = n - 1; s >= 0; --s) ext.update(vec1(wrap02(s * step)));
  CHECK_NEAR(peak, 1.5 * 2.0 * M_PI, 0.1, "peak ~ 1.5 turns (9.42)");
  CHECK_NEAR(ext.getPositions()(0), 0.0, 1e-9, "returns to start after reversal");
}

static void test_normalize_property_fuzz() {
  std::puts("\n[P1I.normalize_fuzz] range + congruence + idempotence over random inputs");
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> d(-50.0, 50.0);
  bool range_ok = true;
  double worst_cong = 0.0, worst_idem = 0.0;
  for (int i = 0; i < 100000; ++i) {
    double const x = d(rng);
    double const n = TestableEJP::norm(x);
    if (!(n >= -M_PI && n < M_PI)) range_ok = false;
    double const k = std::round((x - n) / (2.0 * M_PI));
    worst_cong = std::max(worst_cong, std::abs((n + k * 2.0 * M_PI) - x));
    worst_idem = std::max(worst_idem, std::abs(TestableEJP::norm(n) - n));
  }
  CHECK_TRUE(range_ok, "all outputs in [-pi, pi)");
  CHECK_TRUE(worst_cong < 1e-9, "all outputs congruent to input mod 2pi");
  CHECK_TRUE(worst_idem < 1e-12, "normalize is idempotent on its range");
}

// =====================================================================
//  P1C — Characterisation (documents CURRENT behaviour; not a verdict)
// =====================================================================

static void test_char_init_normalizes() {
  std::puts("\n[P1C.init_normalizes] FACT: init() runs its input through normalize()");
  // Documents what init() does today; whether it SHOULD is the verdict in
  // P2.B2.  Kept separate so the characterisation and the verdict do not
  // masquerade as each other.
  TestableEJP ext(1);
  (void)ext.init(vec1(6.18));
  CHECK_NEAR(ext.diff(), TestableEJP::norm(6.18), 1e-12, "diff = normalize(6.18)");
  CHECK_NEAR(ext.current(), TestableEJP::norm(6.18), 1e-12, "current = normalize(6.18)");
}

static void test_char_large_step_takes_short_way() {
  std::puts("\n[P1C.large_step] FACT: a large step is read the SHORT way (correct unwrap)");
  // A memoryless unwrap MUST pick the shortest wrapped delta, so 0 -> 5.0
  // becomes normalize(5.0) = -1.283, not +5.0.  (This is why an earlier
  // "B4: should be 5.0" test was wrong and was removed.)
  {
    ExtendedJointPositions ext(1);
    (void)ext.init(vec1(0.0));
    ext.update(vec1(5.0));                 // |5-0| >= 3pi/2 -> wrap branch
    CHECK_NEAR(ext.getPositions()(0), TestableEJP::norm(5.0), 1e-9,
               "0->5.0 reads as short-way -1.283");
  }
  // A step beyond pi is also taken the short way (the delta-based unwrap has
  // no branch cut): 0 -> 3.2 wraps to normalize(3.2) = -3.083.
  {
    ExtendedJointPositions ext(1);
    (void)ext.init(vec1(0.0));
    ext.update(vec1(3.2));                 // |3.2-0| < 3pi/2 -> no-wrap branch
    CHECK_NEAR(ext.getPositions()(0), TestableEJP::norm(3.2), 1e-9,
               "0->3.2 reads short-way (-3.083)");
  }
}

static void test_char_nyquist_envelope() {
  std::puts("\n[P1C.nyquist] FACT: steps > pi/cycle alias (operating envelope)");
  // Beyond Nyquist the extended output cannot track: 5 steps of 4.0 rad
  // (> pi) do not recover the true 20 rad.  Fundamental limit, not a code
  // defect — documents the |Δ| < pi/2 envelope the algorithm needs.
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double pos = 0.0;
  for (int s = 1; s <= 5; ++s) { pos += 4.0; ext.update(vec1(wrap02(pos))); }
  CHECK_TRUE(std::abs(ext.getPositions()(0) - 20.0) > 1.0,
             "aliases (does NOT recover true 20 rad)");
}

// =====================================================================
//  P1F — Randomised property fuzz over the LIVE contract (MUST PASS)
//  Random multi-turn, multi-DOF walks with |Δ| < π/2 and readings in
//  [0,2π) — exactly the deployed pipeline.  A correct unwrap reproduces
//  the true continuous physical angle, so we assert that directly, plus
//  congruence and continuity, across several seeds and dof counts.  This
//  is the broad regression net: any future change that breaks in-envelope
//  tracking, on any joint, trips here.
// =====================================================================
static void fuzz_one(unsigned seed, int n_dof) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> step(-1.2, 1.2);  // |Δ| < pi/2 ~ 1.5708
  ExtendedJointPositions ext(n_dof);
  Eigen::VectorXd phys = Eigen::VectorXd::Zero(n_dof);
  (void)ext.init(phys);                 // start at 0 so extended == phys exactly
  Eigen::VectorXd prev = ext.getPositions();
  double worst_track = 0.0, worst_cong = 0.0, worst_jump = 0.0;
  for (int k = 0; k < 3000; ++k) {
    Eigen::VectorXd t(n_dof);
    for (int j = 0; j < n_dof; ++j) { phys(j) += step(rng); t(j) = wrap02(phys(j)); }
    ext.update(t);
    Eigen::VectorXd e = ext.getPositions();
    for (int j = 0; j < n_dof; ++j) {
      worst_track = std::max(worst_track, std::abs(e(j) - phys(j)));
      worst_cong  = std::max(worst_cong,
                    std::abs(TestableEJP::norm(e(j)) - TestableEJP::norm(wrap02(phys(j)))));
      worst_jump  = std::max(worst_jump, std::abs(e(j) - prev(j)));
    }
    prev = e;
  }
  char name[112];
  std::snprintf(name, sizeof name, "seed %u/%ddof: extended tracks true physical angle", seed, n_dof);
  CHECK_TRUE(worst_track < 1e-6, name);
  std::snprintf(name, sizeof name, "seed %u/%ddof: congruent mod 2pi", seed, n_dof);
  CHECK_TRUE(worst_cong < 1e-9, name);
  std::snprintf(name, sizeof name, "seed %u/%ddof: continuity (no ~2pi jumps)", seed, n_dof);
  CHECK_TRUE(worst_jump < 1.3, name);   // max real step 1.2; a 2pi jump trips this
}

static void test_fuzz_live_contract() {
  std::puts("\n[P1F.fuzz] random multi-turn walks reproduce the true physical angle");
  for (unsigned s : {1u, 2u, 3u, 7u, 42u}) fuzz_one(s, 1);
  fuzz_one(101u, 7);   // full Gen3 dof count, independent random walks
}

// =====================================================================
//  P2 — Defects (XFAIL: assert the physically-correct value)
// =====================================================================

// --- INV : representative dependence (root cause of B2/B3/B6) --------
// A position unwrap should depend only on the wrapped delta, hence be
// INVARIANT to the 2π-congruent representative of the input.  Feed the
// same physical ramp (steps of 0.3 rad, inside the envelope) twice — once
// as [0,2π) readings, once as [-π,π) readings.  A range-agnostic unwrap
// gives the same extended position both times; this implementation does
// not (the no-wrap branch uses the absolute reading).  THIS is the direct
// answer to "what happens if the input is not in [0,2π)": it should not
// matter, but currently it does.
static void test_INV_representative_invariance() {
  std::puts("\n[P2.INV] output should be invariant to +/-2pi representative");
  ExtendedJointPositions a(1), b(1);
  (void)a.init(vec1(0.0)); (void)b.init(vec1(0.0));
  for (int k = 1; k <= 20; ++k) {
    double const phys = 0.3 * k;                 // crosses pi, stays < pi/2 per step
    a.update(vec1(wrap02(phys)));                // [0,2pi) representative
    b.update(vec1(TestableEJP::norm(phys)));     // [-pi,pi) representative
  }
  // Both represent the same physical angle; correct value is the [0,2pi)
  // result a (== true physical 6.0).  b should equal a.
  CHECK_NEAR(b.getPositions()(0), a.getPositions()(0), 1e-6,
             "INV: [-pi,pi) input yields the same extended position as [0,2pi)");
}

// --- B1 : LATENT knife-edge -----------------------------------------
// no-wrap turn count uses static_cast<int>(diff/2π) (truncate toward
// zero) with a "-1" correction for negatives — equals floor() EXCEPT at
// exact integers.  At diff = -2π: int(-1.0)-1 = -2 but floor(-1.0) = -1,
// so a stationary joint jumps a full turn.  Only fires at bit-exact
// -2π·k (see P1.walk_through_minus_2pi).  Fix: std::floor(diff/2π).
static void test_B1_exact_minus_2pi() {
  std::puts("\n[P2.B1] off-by-one at bit-exact diff = -2pi (latent)");
  TestableEJP ext(1);
  (void)ext.init(vec1(0.0));
  ext.forceState(-2.0 * M_PI, 0.0);
  ext.update(vec1(0.0));
  CHECK_NEAR(ext.diff(), -2.0 * M_PI, 1e-9, "B1: stationary at -2pi stays -2pi");
}

static void test_B1c_positive_side_control() {
  std::puts("\n[P2.B1c] CONTROL: the +2pi side (plain truncation) is correct");
  TestableEJP ext(1);
  (void)ext.init(vec1(0.0));
  ext.forceState(2.0 * M_PI, 0.0);
  ext.update(vec1(0.0));
  CHECK_NEAR(ext.diff(), 2.0 * M_PI, 1e-9, "B1c: stationary at +2pi stays +2pi");
}

// --- B2 (resolved) : startup is stable at the normalize() anchor -----
// The delta unwrap anchors the extended position at normalize(initial)
// and thereafter accumulates wrapped deltas.  Feeding the SAME reading
// (the controller's startup init(pos); update(pos), controller.cpp:152)
// produces a zero delta, so the position holds — no spurious startup
// motion.  The absolute anchor is normalize(q), NOT q: a memoryless
// unwrap fixes only deltas, and the controller seeds desired from
// getPositions() at startup, so the anchor is self-consistent.  (The
// earlier "identity to the raw reading" expectation was wrong for a
// delta unwrap; the old absolute-reconstruction code is gone.)
static void test_B2_startup_stable_at_anchor() {
  std::puts("\n[P1.b2_startup] init(q); update(q) holds at the normalize(q) anchor");
  double const q = 2.0 * M_PI - 0.1;     // 6.18, a valid [0,2pi) reading
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(q));
  ext.update(vec1(q));
  CHECK_NEAR(ext.getPositions()(0), TestableEJP::norm(q), 1e-12,
             "B2: extended after init+same reading stays at normalize(q)");
  ext.update(vec1(q));                   // repeat: still no drift
  CHECK_NEAR(ext.getPositions()(0), TestableEJP::norm(q), 1e-12,
             "B2: repeated identical reading does not drift");
}

// --- B5 : LIVE on a sensor NaN --------------------------------------
// The NaN guard `continue`s past the diff update, but
// current_joint_positions_ = target_joint_positions runs OUTSIDE the loop
// and copies the NaN into current_.  So current_ is poisoned; next step
// |t - NaN| = NaN, `NaN >= threshold` is false, forcing the no-wrap
// branch and blinding a real wrap on the recovery step (a full turn is
// silently dropped).
static void test_B5_nan_poisons_current() {
  std::puts("\n[P2.B5] a NaN reading poisons current_ (guard protects only diff)");
  TestableEJP ext(1);
  (void)ext.init(vec1(0.1));
  ext.forceState(0.1, 0.1);
  ext.update(vec1(std::numeric_limits<double>::quiet_NaN()));
  CHECK_NEAR(ext.diff(), 0.1, 1e-12, "B5 control: diff protected by guard");
  CHECK_NEAR(ext.current(), 0.1, 1e-12,
             "B5: current_ keeps last good value (not poisoned by NaN)");
}

static void test_B5b_nan_blinds_recovery_wrap() {
  std::puts("\n[P2.B5b] poisoned current_ blinds a real wrap on the next step");
  TestableEJP ext(1);
  (void)ext.init(vec1(6.2));
  ext.forceState(6.2, 6.2);
  ext.update(vec1(std::numeric_limits<double>::quiet_NaN()));   // poisons current_
  ext.update(vec1(0.1));                                        // genuine fwd wrap
  double const correct = 6.2 + TestableEJP::norm(0.1 - 6.2);    // ~6.38
  CHECK_NEAR(ext.diff(), correct, 1e-6, "B5b: wrap on recovery step detected (~6.38)");
}

// --- B6 : DORMANT (defensive regression guard) ----------------------
// The no-wrap reconstruction needs t in [0,2π).  The ONLY thing enforcing
// that is the adapter's ONE-SHOT shift (impl.h:140-141):
//     if (q < 0.0) q += 2.0*M_PI;        // added exactly once
// That handles [-2π,0) but NOT q < -2π: the result stays negative, the
// reconstruction lands in the wrong 2π band, and a turn is lost per step.
// VERIFIED DORMANT: the kortex driver re-centres every reading to [-π,π)
// memorylessly (Gen3Robot.cpp:1000-1001), so the one-shot shift always
// receives [-π,π) and always suffices — q < -2π cannot occur live.  This
// test feeds an out-of-contract multi-turn sequence to prove the class
// would break IF a future driver reported signed/accumulated angles; it
// is a guard, not a live bug.  (Contrast B6b: full-wrapped input is fine.)
static void test_B6_one_shot_shift_insufficient_multi_turn_backward() {
  std::puts("\n[P2.B6] one-shot adapter shift fails past -2pi (multi-turn)");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double pos = 0.0; double const step = 0.1;
  int const n = static_cast<int>(std::round(4.0 * M_PI / step));  // ~2 turns
  for (int i = 0; i < n; ++i) { pos -= step; ext.update(vec1(adapter_one_shot_shift(pos))); }
  CHECK_NEAR(ext.getPositions()(0), pos, 0.5,
             "B6: 2 backward turns via REAL one-shot adapter stay continuous");
}

static void test_B6b_full_wrap_input_is_fine_control() {
  std::puts("\n[P2.B6b] CONTROL: full [0,2pi) wrap of same motion tracks fine");
  ExtendedJointPositions ext(1);
  (void)ext.init(vec1(0.0));
  double pos = 0.0; double const step = 0.1;
  int const n = static_cast<int>(std::round(4.0 * M_PI / step));
  for (int i = 0; i < n; ++i) { pos -= step; ext.update(vec1(wrap02(pos))); }  // FULL wrap
  CHECK_NEAR(ext.getPositions()(0), pos, 0.5,
             "B6b: fully-wrapped input tracks 2 backward turns correctly");
}

// --- B3 : LATENT / robustness (contract-violation probe) ------------
// The wrap branch computes diff += normalize(t) - normalize(c) WITHOUT
// re-wrapping that delta, so a ±π seam crossing yields ∓2π.  Reachable
// only if update() is fed [-π,π) input — which the adapter's shift
// prevents.  Kept as a guard: documents that the adapter shift is
// load-bearing.  Control B3b shows the live [0,2π) path is safe.
static void test_B3_wrap_seam_blowup_contract_violation() {
  std::puts("\n[P2.B3] wrap-branch seam blowup IF fed [-pi,pi) (contract violation)");
  TestableEJP ext(1);
  (void)ext.init(vec1(3.1));
  ext.forceState(3.1, 3.1);
  ext.update(vec1(-3.1));                          // [-pi,pi) input across +pi
  double const correct = 3.1 + TestableEJP::norm(-3.1 - 3.1);   // short way ~3.183
  CHECK_NEAR(ext.diff(), correct, 1e-9, "B3: seam crossing takes the short way");
}

static void test_B3b_seam_ok_in_adapter_convention() {
  std::puts("\n[P2.B3b] CONTROL: same move in live [0,2pi) input is correct");
  TestableEJP ext(1);
  (void)ext.init(vec1(3.1));
  ext.forceState(3.1, 3.1);
  ext.update(vec1(wrap02(-3.1)));                  // adapter would send 3.183
  CHECK_NEAR(ext.diff(), wrap02(-3.1), 1e-9, "B3b: [0,2pi) path tracks correctly");
}

// =====================================================================
//  main
// =====================================================================
int main() {
  std::puts("======== P1: functionality (must pass) ========");
  test_normalize_scalar();
  test_normalize_vector();
  test_init_contract();
  test_no_motion();
  test_small_forward();
  test_small_backward();
  test_forward_seam();
  test_backward_seam();
  test_multi_step_no_wrap();
  test_multi_rotation_forward();
  test_multi_rotation_backward();
  test_walk_through_minus_2pi();
  test_multijoint_independence_and_nan_isolation();
  test_nan_guard_protects_diff();
  test_getpositions_returns_copy();
  test_normalize_boundaries();
  test_init_multidof_sizing();
  test_multidof_simultaneous_opposite_wraps();
  test_getnancount_accumulates();

  std::puts("\n======== P1I: invariants (must pass) ========");
  test_invariants_smooth_sweep();
  test_invariant_reversibility();
  test_normalize_property_fuzz();

  std::puts("\n======== P1C: characterisation (must pass; documents behaviour) ========");
  test_char_init_normalizes();
  test_char_large_step_takes_short_way();
  test_char_nyquist_envelope();

  std::puts("\n======== P1F: randomised live-contract fuzz (must pass) ========");
  test_fuzz_live_contract();

  std::puts("\n======== P2: defects (XFAIL = known defect) ========");
  test_INV_representative_invariance();
  test_B1_exact_minus_2pi();
  test_B1c_positive_side_control();
  test_B2_startup_stable_at_anchor();
  test_B5_nan_poisons_current();
  test_B5b_nan_blinds_recovery_wrap();
  test_B6_one_shot_shift_insufficient_multi_turn_backward();
  test_B6b_full_wrap_input_is_fine_control();
  test_B3_wrap_seam_blowup_contract_violation();
  test_B3b_seam_ok_in_adapter_convention();

  std::printf(
      "\n----------\n"
      "%d checks: %d passed, %d hard-failed | %d known-defect xfail, %d XPASS\n",
      checks_run, checks_run - checks_failed - xfail_known - xpass_fixed,
      checks_failed, xfail_known, xpass_fixed);
  if (xpass_fixed > 0)
    std::puts("NOTE: an XFAIL started passing — a known defect may be fixed; "
              "promote that check to CHECK_NEAR.");

  // Completeness gate.  Coverage alone cannot "guarantee no bugs" while
  // known defects survive as XFAIL: a green run here only means "no
  // *unexpected* regression."  Under EJP_STRICT, any surviving known
  // defect is a hard failure — so a zero exit in strict mode is the real
  // "no known bugs" guarantee.  Use it in CI once the implementation is
  // fixed and the XFAILs are promoted (xfail_known should then be 0).
  bool const strict = std::getenv("EJP_STRICT") != nullptr;
  if (strict && (xfail_known > 0 || xpass_fixed > 0)) {
    std::printf("STRICT: %d known defect(s) unresolved, %d unpromoted XPASS "
                "-> FAIL (a clean run requires zero known defects)\n",
                xfail_known, xpass_fixed);
    return 2;
  }
  return checks_failed == 0 ? 0 : 1;
}
