// GPS lives under the SimulationRun component per the PDF. These tests exercise the
// two-view-over-one-truth MockGPS (the gps_resolution feature).
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/Units.h>

#include <gtest/gtest.h>

#include <memory>

namespace {
using namespace drone_mapper;

std::shared_ptr<GpsTruth> makeTruth(double x, double y, double z, double h, double a) {
    auto t = std::make_shared<GpsTruth>();
    t->position = Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
    t->heading = Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
    return t;
}
Position3D P(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}
Orientation O(double h, double a) {
    return Orientation{h * horizontal_angle[deg], a * altitude_angle[deg]};
}
double X(const Position3D& p) { return p.x.force_numerical_value_in(cm); }
double Y(const Position3D& p) { return p.y.force_numerical_value_in(cm); }
double Z(const Position3D& p) { return p.z.force_numerical_value_in(cm); }
double H(const Orientation& o) { return o.horizontal.force_numerical_value_in(deg); }
double A(const Orientation& o) { return o.altitude.force_numerical_value_in(deg); }
} // namespace

// (a) rounded view rounds to NEAREST multiple (incl. 18->20 up, and a negative axis).
TEST(SimulationRun, GpsRoundedViewRoundsToNearest) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS rounded(truth, 5.0 * cm);
    const Position3D p = rounded.position();
    EXPECT_DOUBLE_EQ(X(p), 15.0); // 17 -> 15
    EXPECT_DOUBLE_EQ(Y(p), 20.0); // 18 -> 20  (nearest, not floor)
    EXPECT_DOUBLE_EQ(Z(p), 10.0); // 12 -> 10

    rounded.setPosition(P(-17, 18, 12));
    EXPECT_DOUBLE_EQ(X(rounded.position()), -15.0); // -17 -> -15
}

// (b) exact view returns the raw position (resolution <= 0).
TEST(SimulationRun, GpsExactViewReturnsRaw) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    const Position3D p = exact.position();
    EXPECT_DOUBLE_EQ(X(p), 17.0);
    EXPECT_DOUBLE_EQ(Y(p), 18.0);
    EXPECT_DOUBLE_EQ(Z(p), 12.0);
}

// (c) one shared truth, two views: a single update reflects on both, no divergence.
TEST(SimulationRun, GpsSharedTruthSingleUpdateBothViews) {
    auto truth = makeTruth(17, 18, 12, 0, 0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);

    rounded.setPose(P(22, 23, 24), O(90, 0)); // update through ONE view

    const Position3D pe = exact.position();
    EXPECT_DOUBLE_EQ(X(pe), 22.0);
    EXPECT_DOUBLE_EQ(Y(pe), 23.0);
    EXPECT_DOUBLE_EQ(Z(pe), 24.0);

    const Position3D pr = rounded.position();
    EXPECT_DOUBLE_EQ(X(pr), 20.0); // 22 -> 20
    EXPECT_DOUBLE_EQ(Y(pr), 25.0); // 23 -> 25
    EXPECT_DOUBLE_EQ(Z(pr), 25.0); // 24 -> 25
}

// (d) heading() is unrounded on both views.
TEST(SimulationRun, GpsHeadingUnrounded) {
    auto truth = makeTruth(0, 0, 0, 47.0, 13.0);
    MockGPS exact(truth, 0.0 * cm);
    MockGPS rounded(truth, 5.0 * cm);
    EXPECT_DOUBLE_EQ(H(exact.heading()), 47.0);
    EXPECT_DOUBLE_EQ(A(exact.heading()), 13.0);
    EXPECT_DOUBLE_EQ(H(rounded.heading()), 47.0);
    EXPECT_DOUBLE_EQ(A(rounded.heading()), 13.0);
}
