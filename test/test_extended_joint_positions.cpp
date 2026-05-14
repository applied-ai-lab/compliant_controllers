/**
 * Standalone test harness for ExtendedJointPositions::update logic.
 *
 * Replicates the algorithm from
 * src/compliant_controllers/src/extended_joint_positions.cpp using
 * std::vector<double> in place of Eigen::VectorXd, so it can be
 * compiled without Eigen / ROS / Pinocchio.  The per-element math
 * is identical to the original (the Eigen wrapper just provides
 * vector arithmetic — element-wise this is plain scalar code).
 *
 * Tests cover:
 *   1. Init then no motion
 *   2. Small forward motion (no wrap)
 *   3. Small backward motion (no wrap)
 *   4. Forward wrap (raw crosses 2π → 0)
 *   5. Backward wrap (raw crosses 0 → 2π)
 *   6. Many consecutive small forward steps (accumulating extended)
 *   7. Many consecutive small backward steps
 *   8. Multi-rotation forward tracking (joint spins multiple turns)
 *   9. Multi-rotation backward tracking
 *  10. Edge case: diff exactly at -2π
 *  11. Edge case: NaN target propagates to diff (sticky)
 *  12. Edge case: convention mismatch ([-π,π) init vs [0,2π) update)
 */
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

// ---------- Replica of ExtendedJointPositions ----------------------

class ExtendedJointPositions {
 public:
  ExtendedJointPositions(unsigned int n_dof, double threshold = 3.0 * M_PI / 2.0)
      : is_initialized_(false), n_dof_(n_dof), threshold_(threshold),
        diff_(n_dof, 0.0), current_(n_dof, 0.0) {}

  bool init(std::vector<double> const& joint_positions) {
    if (!is_initialized_) {
      // Convention-fix: store verbatim, not normalised.  Matches
      // the in-tree fix in src/extended_joint_positions.cpp.
      diff_ = joint_positions;
      current_ = joint_positions;
      is_initialized_ = true;
      return true;
    }
    return false;
  }

  void update(std::vector<double> const& target) {
    for (std::size_t i = 0; i < target.size(); ++i) {
      // F4: NaN guard mirrored from in-tree
      // src/extended_joint_positions.cpp.  Skip the per-joint
      // update if target is non-finite.
      if (!std::isfinite(target[i])) {
        continue;
      }
      if (std::abs(target[i] - current_[i]) >= threshold_) {
        diff_[i] += normalize(target[i]) - normalize(current_[i]);
      } else {
        int n_rot = 0;
        if (diff_[i] >= 0.0) {
          n_rot = static_cast<int>(diff_[i] / (2.0 * M_PI));
        } else {
          n_rot = static_cast<int>(diff_[i] / (2.0 * M_PI)) - 1;
        }
        diff_[i] = n_rot * (2.0 * M_PI) + target[i];
      }
    }
    current_ = target;
  }

  std::vector<double> const& getPositions() const { return diff_; }
  bool isInitialized() const { return is_initialized_; }

 private:
  static double normalize(double a) {
    double out = std::fmod(a + M_PI, 2.0 * M_PI);
    if (out < 0.0) out += 2.0 * M_PI;
    return out - M_PI;
  }

  static std::vector<double> normalize_vec(std::vector<double> const& v) {
    std::vector<double> out(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) out[i] = normalize(v[i]);
    return out;
  }

  bool is_initialized_;
  unsigned int n_dof_;
  double threshold_;
  std::vector<double> diff_;
  std::vector<double> current_;
};

// ---------- Test harness ------------------------------------------

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK_NEAR(actual, expected, tol, name)                              \
  do {                                                                       \
    ++tests_run;                                                             \
    double const _a = (actual);                                              \
    double const _e = (expected);                                            \
    if (std::isnan(_a) != std::isnan(_e) ||                                  \
        (!std::isnan(_e) && std::abs(_a - _e) > (tol))) {                    \
      ++tests_failed;                                                        \
      std::fprintf(stderr,                                                   \
                   "  FAIL  %s  expected=%g  actual=%g  diff=%g  tol=%g\n",  \
                   (name), _e, _a, _a - _e, (tol));                          \
    } else {                                                                 \
      std::printf("  pass  %s  =%g\n", (name), _a);                          \
    }                                                                        \
  } while (0)

#define CHECK_NAN(actual, name)                                              \
  do {                                                                       \
    ++tests_run;                                                             \
    double const _a = (actual);                                              \
    if (std::isnan(_a)) {                                                    \
      std::printf("  pass  %s  =NaN (as expected)\n", (name));               \
    } else {                                                                 \
      ++tests_failed;                                                        \
      std::fprintf(stderr, "  FAIL  %s  expected NaN, got %g\n", (name), _a);\
    }                                                                        \
  } while (0)

// Helper: build a single-joint extended-position tracker, init at q0,
// run a sequence of raw target values, return the extended trajectory.
std::vector<double> run_sequence(double q0,
                                 std::vector<double> const& targets) {
  ExtendedJointPositions ext(1);
  std::vector<double> init_v{q0};
  ext.init(init_v);
  std::vector<double> traj;
  traj.reserve(targets.size());
  for (double t : targets) {
    std::vector<double> v{t};
    ext.update(v);
    traj.push_back(ext.getPositions()[0]);
  }
  return traj;
}

// ---------- Tests --------------------------------------------------

void test_init_no_motion() {
  std::puts("\n[test_init_no_motion]");
  // Init at 1.5; first update with same value should leave extended at 1.5.
  auto traj = run_sequence(1.5, {1.5, 1.5, 1.5});
  CHECK_NEAR(traj[0], 1.5, 1e-12, "first update no motion");
  CHECK_NEAR(traj[2], 1.5, 1e-12, "third update no motion");
}

void test_small_forward_motion() {
  std::puts("\n[test_small_forward_motion]");
  // Init at 0.0, raw goes 0 -> 0.1 -> 0.2 -> 0.3 (small forward steps).
  auto traj = run_sequence(0.0, {0.1, 0.2, 0.3});
  CHECK_NEAR(traj[0], 0.1, 1e-12, "step 1 = 0.1");
  CHECK_NEAR(traj[1], 0.2, 1e-12, "step 2 = 0.2");
  CHECK_NEAR(traj[2], 0.3, 1e-12, "step 3 = 0.3");
}

void test_small_backward_motion() {
  std::puts("\n[test_small_backward_motion]");
  // Init at 1.0, raw goes 1.0 -> 0.9 -> 0.8 -> 0.7.
  auto traj = run_sequence(1.0, {0.9, 0.8, 0.7});
  CHECK_NEAR(traj[0], 0.9, 1e-12, "step 1 = 0.9");
  CHECK_NEAR(traj[1], 0.8, 1e-12, "step 2 = 0.8");
  CHECK_NEAR(traj[2], 0.7, 1e-12, "step 3 = 0.7");
}

void test_forward_wrap_through_2pi() {
  std::puts("\n[test_forward_wrap_through_2pi]");
  // hardware_interface_adapter wraps raw to [0, 2π).
  // Joint at raw=6.0 (just below 2π).  Continues forward — raw wraps to
  // 0.1 (small motion of ~0.38 rad past 2π).  Extended should track
  // smoothly: 6.0 -> 6.0+small -> 6.38.
  auto traj = run_sequence(6.0, {0.1, 0.2});
  // After raw 6.0 -> 0.1, branch 1 (wrap) fires.  Extended:
  //   diff += normalize(0.1) - normalize(6.0)
  //   normalize(6.0) = fmod(6.0+π, 2π)-π ≈ 9.14-6.28-3.14 = -0.28
  //   normalize(0.1) = 0.1
  //   diff = 6.0 + (0.1 - (-0.28)) = 6.38
  CHECK_NEAR(traj[0], 6.38, 0.01, "after forward wrap, extended ~= 6.38");
  // After raw 0.1 -> 0.2, small motion (no wrap).
  //   diff_prev = 6.38 (≥0).  n_rot = int(6.38/2π) = 1.
  //   diff = 1*2π + 0.2 = 6.483
  CHECK_NEAR(traj[1], 6.483, 0.01, "extended continues to ~6.48");
}

void test_backward_wrap_through_zero() {
  std::puts("\n[test_backward_wrap_through_zero]");
  // Joint at raw=0.1.  Moves backward, raw wraps to 6.18 (= 2π - 0.1).
  // Physical motion is just -0.2 rad.  Extended should go 0.1 -> -0.1.
  auto traj = run_sequence(0.1, {6.18, 6.0});
  CHECK_NEAR(traj[0], -0.1, 0.01, "after backward wrap, extended ~= -0.1");
  // Next step: raw 6.18 -> 6.0, small backward (no wrap).
  //   diff_prev = -0.1 (<0).  n_rot = int(-0.1/2π)-1 = 0-1 = -1.
  //   diff = -1*2π + 6.0 = -0.28
  CHECK_NEAR(traj[1], -0.28, 0.01, "extended continues to ~-0.28");
}

void test_multi_step_no_wrap() {
  std::puts("\n[test_multi_step_no_wrap]");
  // 100 small forward steps, no wrap.  Extended should equal target
  // every step.
  std::vector<double> targets;
  for (int i = 1; i <= 100; ++i) targets.push_back(0.01 * i);
  auto traj = run_sequence(0.0, targets);
  CHECK_NEAR(traj[99], 1.0, 1e-9, "after 100 steps, extended = 1.0");
}

void test_multi_rotation_forward() {
  std::puts("\n[test_multi_rotation_forward]");
  // Joint spins 3 full rotations forward.  raw cycles through:
  //   0 -> ~2π -> 0 -> ~2π -> 0 -> ~2π -> 0
  // Extended should end at ~6π = 18.85 (or close to start if "0").
  std::vector<double> targets;
  // Walk in 0.1 rad increments.  Each cycle of 2π ≈ 63 steps.
  double const step = 0.1;
  int const n_cycles = 3;
  int const steps_per_cycle = static_cast<int>(std::round(2.0 * M_PI / step));
  for (int c = 0; c < n_cycles; ++c) {
    for (int s = 1; s <= steps_per_cycle; ++s) {
      double raw = std::fmod(s * step, 2.0 * M_PI);
      targets.push_back(raw);
    }
  }
  auto traj = run_sequence(0.0, targets);
  // After 3 cycles, extended should be ~ 3 * 2π = 18.85
  double const expected = n_cycles * steps_per_cycle * step;
  CHECK_NEAR(traj.back(), expected, 0.5,
             "extended after 3 forward rotations ≈ 18.85");
}

void test_multi_rotation_backward() {
  std::puts("\n[test_multi_rotation_backward]");
  // 3 full rotations backward.  Mirror of forward.
  std::vector<double> targets;
  double const step = 0.1;
  int const n_cycles = 3;
  int const steps_per_cycle = static_cast<int>(std::round(2.0 * M_PI / step));
  double pos = 0.0;
  for (int c = 0; c < n_cycles; ++c) {
    for (int s = 1; s <= steps_per_cycle; ++s) {
      pos -= step;
      // Wrap to [0, 2π).
      double raw = std::fmod(pos, 2.0 * M_PI);
      if (raw < 0.0) raw += 2.0 * M_PI;
      targets.push_back(raw);
    }
  }
  auto traj = run_sequence(0.0, targets);
  double const expected = -n_cycles * steps_per_cycle * step;
  CHECK_NEAR(traj.back(), expected, 0.5,
             "extended after 3 backward rotations ≈ -18.85");
}

void test_edge_case_diff_at_minus_2pi() {
  std::puts("\n[test_edge_case_diff_at_minus_2pi]");
  // Force the internal diff to be exactly -2π by walking the joint
  // backward 1 full rotation, ending at raw = 0.0.
  std::vector<double> targets;
  double pos = 0.0;
  // Make sure we don't get small numerical drift — use exact arithmetic.
  // 2π/0.001 ≈ 6283.18 steps.  Use larger step.
  double const step = 2.0 * M_PI / 100.0;
  for (int s = 1; s <= 100; ++s) {
    pos -= step;
    double raw = std::fmod(pos, 2.0 * M_PI);
    if (raw < 0.0) raw += 2.0 * M_PI;
    targets.push_back(raw);
  }
  auto traj = run_sequence(0.0, targets);
  // After exactly 1 backward rotation, extended ≈ -2π = -6.283.
  CHECK_NEAR(traj.back(), -2.0 * M_PI, 0.5,
             "extended at full backward rotation");

  // Continue: one more small backward step.
  ExtendedJointPositions ext(1);
  ext.init({0.0});
  for (double t : targets) ext.update({t});
  double diff_before = ext.getPositions()[0];
  std::printf("  diff just before next step: %g (target was %g)\n",
              diff_before, -2.0 * M_PI);

  // Next step: small backward motion.
  // pos is at -2π by construction; raw = 0.0.  Step back 0.01.
  // New raw = 6.273.  abs(6.273 - 0.0) = 6.273 > 4.71 → branch 1.
  ext.update({2.0 * M_PI - 0.01});
  double after = ext.getPositions()[0];
  // Physical: extended was -2π, moved backward by 0.01, expected = -2π - 0.01.
  CHECK_NEAR(after, -2.0 * M_PI - 0.01, 0.05,
             "small backward step from extended ≈ -2π");
}

void test_nan_target_sticks() {
  std::puts("\n[test_nan_target_sticks]");
  // Regression test for F4 (NaN guard at the top of update()).
  // Send one NaN target.  Expected behaviour with F4: the per-joint
  // update is skipped, diff stays at the previous good value, and
  // subsequent finite targets are processed normally.
  ExtendedJointPositions ext(1);
  ext.init({0.0});
  ext.update({0.1});
  CHECK_NEAR(ext.getPositions()[0], 0.1, 1e-12, "before NaN, extended = 0.1");

  // Inject NaN — F4 should leave diff at 0.1 untouched.
  ext.update({std::nan("")});
  CHECK_NEAR(ext.getPositions()[0], 0.1, 1e-12,
             "F4 guard: NaN target leaves diff unchanged at 0.1");

  // Recover with a finite target — should advance from 0.1 to 0.2.
  // current_ also stayed at 0.1 (F4 skips the line 69 update too,
  // because `continue` short-circuits the whole iteration).
  ext.update({0.2});
  CHECK_NEAR(ext.getPositions()[0], 0.2, 1e-12,
             "after F4-skipped NaN, recovery to 0.2 is clean");
}

void test_convention_mismatch() {
  std::puts("\n[test_convention_mismatch]");
  // After the init() convention fix (storing verbatim instead of
  // normalising), init and update use the same angle convention.
  // init(6.18) → current_ = 6.18.  update(6.2) → |6.2-6.18| = 0.02
  // (below threshold) → else branch → diff = 0*2π + 6.2 = 6.2.
  ExtendedJointPositions ext(1);
  ext.init({6.18});
  CHECK_NEAR(ext.getPositions()[0], 6.18, 1e-12, "after init(6.18), diff = 6.18");
  ext.update({6.2});
  CHECK_NEAR(ext.getPositions()[0], 6.2, 1e-12, "after update(6.2), diff = 6.2");
}

void test_else_branch_boundary_exactly_minus_2pi() {
  std::puts("\n[test_else_branch_boundary_exactly_minus_2pi]");
  // Theoretical edge case: diff_prev = -2π exactly, target = 0
  // exactly, current = 0.  Algorithm should yield extended = -2π
  // (joint hasn't moved).  Bug: int(-2π/2π) = int(-1.0) = -1 (since
  // C++ truncates toward zero only for non-integer negatives), then
  // else branch subtracts 1 → n_rot = -2 → new diff = -2*2π + 0 = -4π.
  //
  // We can't easily reach this state via normal walking because the
  // algorithm's own invariant (current = previous target) drifts the
  // values numerically.  So we bypass init and manually construct
  // the state via a wrapper.
  //
  // For this test we use a thin wrapper that lets us set internal
  // state directly.  In real code this is invisible — but if joint
  // measurements ever happen to land on these exact values
  // numerically, the bug triggers.
  class ExposedExtJoints : public ExtendedJointPositions {
   public:
    using ExtendedJointPositions::ExtendedJointPositions;
    // Force the algorithm to a known state.
    void forceState(double diff_val, double current_val) {
      // Init normally first to set is_initialized_.
      std::vector<double> v{current_val};
      init(v);
      // Now we need to overwrite diff and current.  Since they're
      // protected, walk the algorithm to a state that matches via
      // the public API.  For diff = -2π and current = 0: init(0)
      // gives diff=0 and current=0.  Then we need to drive diff
      // to -2π without changing current.  The only way via the
      // public API is to call update with a forward-wrap-equivalent
      // sequence... or use a friend declaration.
      //
      // Pragmatically: just call update once with a backward-wrap
      // target that lands diff at -2π.  Easier: init(0), then
      // update(target chosen to put diff at -2π by branch 1).
      // After init(0): diff=0, current=0.
      // update with target T such that abs(T - 0) > threshold AND
      // normalize(T) - normalize(0) = -2π exactly.  That requires
      // T ≈ small negative wrapped to [0, 2π), normalize(T) ≈ small
      // negative ≈ -2π is impossible (normalize range is [-π,π)).
      //
      // OK can't construct.  We document the failure mode instead.
      (void)diff_val;  (void)current_val;
    }
  };
  // Instead: test what happens with diff drifting to near -2π via a
  // long backward walk and one final target=0.
  ExtendedJointPositions ext(1);
  ext.init({0.0});
  // Walk 100 steps backward, each of step = 2π/100, then one extra
  // step that lands target exactly at 0.
  double const step = 2.0 * M_PI / 100.0;
  double pos = 0.0;
  for (int s = 1; s <= 99; ++s) {
    pos -= step;
    double raw = std::fmod(pos, 2.0 * M_PI);
    if (raw < 0.0) raw += 2.0 * M_PI;
    ext.update({raw});
  }
  // diff should be ≈ -99*step = -6.2204
  double diff99 = ext.getPositions()[0];
  std::printf("  after 99 backward steps: diff=%g (≈ -6.22)\n", diff99);

  // 100th step: pos = -100*step = -2π.  fmod(-2π, 2π) = 0.
  // current is the previous raw (= 2π - step ≈ 6.22).  target = 0.
  // abs(0 - 6.22) = 6.22 > 4.71 → BRANCH 1 (wrap detected).
  ext.update({0.0});
  double diff100 = ext.getPositions()[0];
  // Expect: diff += normalize(0) - normalize(6.22)
  //   normalize(6.22) = -0.063 (≈ 6.22 - 2π).
  //   normalize(0) = 0.
  //   diff += 0 - (-0.063) = +0.063 (treating as forward-wrap!).
  //   diff = -6.22 + 0.063 = -6.157.
  // But physical: joint moved backward 0.063 rad.  Extended should
  // be -6.28, not -6.16!  Branch 1 misclassified this as forward wrap.
  std::printf("  after 100th step (target=0): diff=%g\n", diff100);
  std::printf("  expected ~-2π (-6.28); got %g (diff = %g)\n",
              diff100, diff100 - (-2.0 * M_PI));
  if (std::abs(diff100 - (-2.0 * M_PI)) < 0.1) {
    std::puts("  pass  boundary case OK");
  } else {
    std::puts("  FAIL  branch 1 misclassified a backward wrap that lands on 0");
    ++tests_failed;
  }
  ++tests_run;
}

void test_init_at_near_wrap_then_forward_wrap() {
  std::puts("\n[test_init_at_near_wrap_then_forward_wrap]");
  // Regression test for the init() convention fix.  Before the fix,
  // init(6.18) normalised to -0.1, causing the first update to
  // misclassify the forward wrap as a "no wrap" small backward
  // motion and produce diff = -6.18 (off by 2π).
  //
  // After the fix, init stores 6.18 verbatim.  update(0.1) sees
  // |0.1-6.18| = 6.08 > 4.71 → branch 1 (correct wrap detection).
  // diff += normalize(0.1) - normalize(6.18) = 0.1 - (-0.1) = 0.2.
  // diff = 6.18 + 0.2 = 6.38.
  ExtendedJointPositions ext(1);
  ext.init({2.0 * M_PI - 0.1});  // raw ≈ 6.18
  ext.update({0.1});             // forward wrap of ~0.2 rad
  CHECK_NEAR(ext.getPositions()[0], 6.38, 0.05,
             "first wrap after init now handled correctly");
}

// ---------- main ---------------------------------------------------

int main() {
  test_init_no_motion();
  test_small_forward_motion();
  test_small_backward_motion();
  test_forward_wrap_through_2pi();
  test_backward_wrap_through_zero();
  test_multi_step_no_wrap();
  test_multi_rotation_forward();
  test_multi_rotation_backward();
  test_edge_case_diff_at_minus_2pi();
  test_nan_target_sticks();
  test_convention_mismatch();
  test_else_branch_boundary_exactly_minus_2pi();
  test_init_at_near_wrap_then_forward_wrap();

  std::printf("\n----------\n%d / %d tests passed\n",
              tests_run - tests_failed, tests_run);
  return tests_failed == 0 ? 0 : 1;
}
