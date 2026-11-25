#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <algorithm>
#include "libslic3r/libslic3r.h"
#include <vector>
#include <cstddef>
#include <string>
#include <cfloat>
#include <cmath>

using namespace Slic3r;

// A sphere gives well over ExtruderMax original facets, so every extruder state can be assigned
// to a facet of its own without any splitting getting in the way.
static TriangleMesh test_mesh() { return make_sphere(5., 2 * PI / 24); }

// Read the nibble_idx-th 4-bit group of a serialized bitstream, least significant bit first.
static int nibble_at(const std::vector<bool> &bitstream, size_t nibble_idx)
{
    int n = 0;
    for (size_t bit = 0; bit < 4; ++bit)
        n |= int(bitstream[nibble_idx * 4 + bit]) << bit;
    return n;
}

TEST_CASE("Every extruder state survives a serialize/deserialize round trip", "[TriangleSelector]")
{
    const TriangleMesh mesh      = test_mesh();
    const int          max_state = int(EnforcerBlockerType::ExtruderMax);
    REQUIRE(int(mesh.its.indices.size()) >= max_state);

    TriangleSelector selector(mesh);
    for (int state = 1; state <= max_state; ++state)
        selector.set_facet(state - 1, EnforcerBlockerType(state));

    TriangleSelector restored(mesh);
    restored.deserialize(selector.serialize());

    for (int state = 1; state <= max_state; ++state) {
        INFO("Extruder " << state);
        REQUIRE(restored.has_facets(EnforcerBlockerType(state)));
        REQUIRE(restored.num_facets(EnforcerBlockerType(state)) == 1);
    }
}

TEST_CASE("Serialized data reports the extruder states it uses", "[TriangleSelector]")
{
    const TriangleMesh mesh = test_mesh();
    TriangleSelector   selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::Extruder16);
    selector.set_facet(1, EnforcerBlockerType::Extruder32);

    const TriangleSelector::TriangleSplittingData data = selector.serialize();

    REQUIRE(data.used_states.size() == size_t(EnforcerBlockerType::ExtruderMax) + 1);
    REQUIRE(data.used_states[size_t(EnforcerBlockerType::Extruder16)]);
    REQUIRE(data.used_states[size_t(EnforcerBlockerType::Extruder32)]);
    REQUIRE_FALSE(data.used_states[size_t(EnforcerBlockerType::Extruder17)]);

    SECTION("used_states recomputed from the bitstream agrees") {
        TriangleSelector::TriangleSplittingData recomputed = data;
        recomputed.reset_used_states();
        recomputed.update_used_states(0);
        REQUIRE(recomputed.used_states == data.used_states);
    }

    SECTION("has_facets on the raw data agrees") {
        REQUIRE(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder32));
        REQUIRE_FALSE(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder17));
    }
}

// States 3..17 must keep the pre-existing encoding ("11" prefix plus one nibble of state-3) so
// projects written by older builds stay readable and newly written ones stay readable by them.
TEST_CASE("Extruder states up to 17 keep the single-nibble encoding", "[TriangleSelector]")
{
    const int state = GENERATE(3, 8, 16, 17);

    TriangleSelector selector(test_mesh());
    selector.set_facet(0, EnforcerBlockerType(state));
    const std::vector<bool> bitstream = selector.serialize().bitstream;

    INFO("Extruder " << state);
    // Two nibbles: the "11"-prefixed leaf code, then the state itself.
    REQUIRE(bitstream.size() == 8);
    REQUIRE(nibble_at(bitstream, 0) == 0b1100);
    REQUIRE(nibble_at(bitstream, 1) == state - 3);
}

// States 18 and above set the state nibble to 0b1111 and carry (state-18) in one more nibble.
TEST_CASE("Extruder states above 17 are encoded in a second nibble", "[TriangleSelector]")
{
    const int state = GENERATE(18, 25, 32);

    TriangleSelector selector(test_mesh());
    selector.set_facet(0, EnforcerBlockerType(state));
    const std::vector<bool> bitstream = selector.serialize().bitstream;

    INFO("Extruder " << state);
    REQUIRE(bitstream.size() == 12);
    REQUIRE(nibble_at(bitstream, 0) == 0b1100);
    REQUIRE(nibble_at(bitstream, 1) == 0b1111);
    REQUIRE(nibble_at(bitstream, 2) == state - 18);
}

// Model.cpp writes these hex strings into the 3MF for colored mesh imports; the selector must
// decode exactly the states CONST_FILAMENTS assigns to them.
TEST_CASE("Extruder states match the CONST_FILAMENTS hex encoding", "[TriangleSelector]")
{
    struct Case { const char *hex; int state; };
    const auto c = GENERATE(values<Case>({
        {"8", 2}, {"0C", 3}, {"DC", 16}, {"EC", 17}, {"0FC", 18}, {"EFC", 32},
    }));

    // get_triangle_as_string emits the nibbles most significant first, so read the hex backwards.
    const std::string hex = c.hex;
    std::vector<bool> bitstream;
    for (auto it = hex.rbegin(); it != hex.rend(); ++it) {
        const int nibble = *it >= 'A' ? (*it - 'A' + 10) : (*it - '0');
        for (int bit = 0; bit < 4; ++bit)
            bitstream.push_back((nibble >> bit) & 1);
    }

    TriangleSelector::TriangleSplittingData data;
    data.triangles_to_split.emplace_back(0, 0);
    data.bitstream = bitstream;

    INFO("Hex " << c.hex << " -> extruder " << c.state);
    REQUIRE(TriangleSelector::has_facets(data, EnforcerBlockerType(c.state)));
}

// Pack 4-bit codes into a bitstream, least significant bit first, in the order the decoder reads them.
static std::vector<bool> pack_nibbles(const std::vector<int> &nibbles)
{
    std::vector<bool> bitstream;
    for (const int nibble : nibbles)
        for (int bit = 0; bit < 4; ++bit)
            bitstream.push_back((nibble >> bit) & 1);
    return bitstream;
}

TEST_CASE("A valid paint stream with nested splits round-trips bit for bit", "[TriangleSelector]")
{
    const TriangleMesh mesh = test_mesh();

    TriangleSelector::TriangleSplittingData data;
    data.triangles_to_split.emplace_back(0, 0);
    // A three-side split whose children, in stream order, are: a one-side split (side 2) into two
    // leaves, a two-side split (side 1) into leaves of states 20, 0 and 8, then two plain leaves.
    const std::vector<int> triangle_0 = {0b0011,
                                         0b1001, 0b1000, 0b0100,
                                         0b0110, 0b1100, 0b1111, 20 - 18, 0b0000, 0b1100, 8 - 3,
                                         0b1000,
                                         0b0100};
    data.bitstream = pack_nibbles(triangle_0);
    data.triangles_to_split.emplace_back(5, int(data.bitstream.size()));
    const std::vector<bool> triangle_5 = pack_nibbles({0b1100, 3 - 3});
    data.bitstream.insert(data.bitstream.end(), triangle_5.begin(), triangle_5.end());
    data.reset_used_states();
    REQUIRE(data.update_used_states(0));

    TriangleSelector restored(mesh);
    restored.deserialize(data);

    REQUIRE(restored.num_facets(EnforcerBlockerType::Extruder20) == 1);
    REQUIRE(restored.num_facets(EnforcerBlockerType::Extruder3) == 1);
    REQUIRE(restored.serialize() == data);
}

TEST_CASE("A truncated or malformed paint stream drops only the damaged triangle", "[TriangleSelector][Regression]")
{
    struct Case { const char *name; std::vector<int> nibbles; };
    const auto c = GENERATE(values<Case>({
        {"three-side split missing two children", {0b0011, 0b1000, 0b1000}},
        {"leaf missing its state nibble",         {0b1100}},
        {"leaf missing its second state nibble",  {0b1100, 0b1111}},
        {"splits nested past the end",            {0b0011, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF,
                                                    0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF, 0xF}},
        {"one-side split of the nonexistent side 3", {0b1101, 0b1000, 0b1000}},
    }));
    INFO(c.name);

    const TriangleMesh mesh = test_mesh();
    TriangleSelector   intact(mesh);
    intact.set_facet(0, EnforcerBlockerType::Extruder2);

    // Triangle 0 stays intact, triangle 1 carries the damaged stream.
    TriangleSelector::TriangleSplittingData data = intact.serialize();
    data.triangles_to_split.emplace_back(1, int(data.bitstream.size()));
    const std::vector<bool> damaged = pack_nibbles(c.nibbles);
    data.bitstream.insert(data.bitstream.end(), damaged.begin(), damaged.end());

    TriangleSelector restored(mesh);
    REQUIRE_NOTHROW(restored.deserialize(data));
    // Triangle 1 unwinds completely, so the selector holds exactly the intact paint.
    REQUIRE(restored.serialize() == intact.serialize());

    REQUIRE_NOTHROW(TriangleSelector::has_facets(data, EnforcerBlockerType::Extruder3));

    TriangleSelector::TriangleSplittingData recomputed = data;
    recomputed.reset_used_states();
    REQUIRE_FALSE(recomputed.update_used_states(0));
    REQUIRE(std::none_of(recomputed.used_states.begin(), recomputed.used_states.end(), [](bool used) { return used; }));
}

// ---------------------------------------------------------------------------------------------------------------
// Border line tool (colour painting): borders stop bucket fill; snapped paths follow sharp edges and grooves.

// An open n x n grid of square cells (each split into two upward-facing triangles), `cell` mm apart. Vertices with
// raise_from <= x, y <= raise_to (in mm) are lifted by `raise_height`, which gives a raised square block whose
// walls slope over one cell.
static TriangleMesh grid_mesh(int n, float cell, float raise_from = 0.f, float raise_to = -1.f, float raise_height = 0.f)
{
    indexed_triangle_set its;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            const float x = i * cell, y = j * cell;
            const bool  raised = x >= raise_from - 1e-4f && x <= raise_to + 1e-4f && y >= raise_from - 1e-4f && y <= raise_to + 1e-4f;
            its.vertices.emplace_back(x, y, raised ? raise_height : 0.f);
        }
    auto idx = [n](int i, int j) { return j * n + i; };
    for (int j = 0; j + 1 < n; ++j)
        for (int i = 0; i + 1 < n; ++i) {
            its.indices.emplace_back(idx(i, j), idx(i + 1, j), idx(i + 1, j + 1));
            its.indices.emplace_back(idx(i, j), idx(i + 1, j + 1), idx(i, j + 1));
        }
    return TriangleMesh(its);
}

// Index of the grid vertex at (x, y) mm.
static int grid_vertex(int n, float cell, float x, float y) { return int(std::lround(y / cell)) * n + int(std::lround(x / cell)); }

// Fill from the triangle whose centre is nearest `point`, edge detection off, and return how many triangles it took.
static int fill_count(TriangleSelector &selector, const TriangleMesh &mesh, const Vec3f &point)
{
    int   best = 0;
    float best_dist = FLT_MAX;
    for (int t = 0; t < int(mesh.its.indices.size()); ++t) {
        const auto &tri    = mesh.its.indices[t];
        const Vec3f centre = (mesh.its.vertices[tri[0]] + mesh.its.vertices[tri[1]] + mesh.its.vertices[tri[2]]) / 3.f;
        if ((centre - point).squaredNorm() < best_dist) {
            best_dist = (centre - point).squaredNorm();
            best      = t;
        }
    }
    const auto &tri    = mesh.its.indices[best];
    const Vec3f centre = (mesh.its.vertices[tri[0]] + mesh.its.vertices[tri[1]] + mesh.its.vertices[tri[2]]) / 3.f;
    selector.seed_fill_unselect_all_triangles();
    selector.bucket_fill_select_triangles(centre, best, TriangleSelector::ClippingPlane(), -1.f, true, true);
    selector.seed_fill_apply_on_triangles(EnforcerBlockerType::Extruder2);
    const int count = selector.num_facets(EnforcerBlockerType::Extruder2);
    selector.reset();
    return count;
}

TEST_CASE("Bucket fill stops at a closed fill border", "[TriangleSelector][BorderLine]")
{
    const int   n    = 21; // 20 x 20 cells of 1 mm = 800 triangles
    const float cell = 1.f;
    const TriangleMesh mesh = grid_mesh(n, cell);
    TriangleSelector   selector(mesh);
    REQUIRE(fill_count(selector, mesh, Vec3f(10.f, 10.f, 0.f)) == 800);

    // A closed square border along the grid lines x = 5, x = 15, y = 5, y = 15.
    std::vector<int> loop;
    for (int i = 5; i <= 15; ++i) loop.push_back(grid_vertex(n, cell, float(i), 5.f));
    for (int j = 6; j <= 15; ++j) loop.push_back(grid_vertex(n, cell, 15.f, float(j)));
    for (int i = 14; i >= 5; --i) loop.push_back(grid_vertex(n, cell, float(i), 15.f));
    for (int j = 14; j >= 5; --j) loop.push_back(grid_vertex(n, cell, 5.f, float(j)));
    selector.add_fill_border(loop);
    REQUIRE(selector.has_fill_borders());
    CHECK(selector.fill_border_edges().size() == 40);

    // reset() clears paint but keeps borders, so both sides can be measured with the same borders.
    CHECK(fill_count(selector, mesh, Vec3f(10.f, 10.f, 0.f)) == 200); // inside: 10 x 10 cells
    CHECK(fill_count(selector, mesh, Vec3f(1.f, 1.f, 0.f)) == 600);   // outside

    SECTION("removing the border lets fill through again") {
        selector.remove_fill_border(loop);
        CHECK_FALSE(selector.has_fill_borders());
        CHECK(fill_count(selector, mesh, Vec3f(10.f, 10.f, 0.f)) == 800);
    }
}

TEST_CASE("Fill borders do not change serialized paint", "[TriangleSelector][BorderLine]")
{
    const TriangleMesh mesh = grid_mesh(11, 1.f);
    TriangleSelector   selector(mesh);
    selector.set_facet(3, EnforcerBlockerType::Extruder3);
    const auto before = selector.serialize();
    selector.add_fill_border({0, 1, 2, 3});
    CHECK(selector.serialize() == before);
    selector.clear_fill_borders();
    CHECK(selector.serialize() == before);
}

TEST_CASE("Snapped path follows the edges of a raised block", "[TriangleSelector][BorderLine]")
{
    // 0.5 mm cells; a 0.5 mm high block from 5 to 15 mm whose walls slope over one cell (4.5 -> 5 mm).
    const int   n    = 41;
    const float cell = 0.5f;
    const TriangleMesh mesh = grid_mesh(n, cell, 5.f, 15.f, 0.5f);
    TriangleSelector   selector(mesh);

    // From a point on the block's bottom edge to a point on its right edge: the snapped path goes round the
    // corner at (15, 5). The grid's diagonals run in the +x+y direction, so the plain shortest path cuts across
    // the top of the block instead (no tie between the two).
    const int from = grid_vertex(n, cell, 10.f, 5.f);
    const int to   = grid_vertex(n, cell, 15.f, 10.f);
    auto on_block_edge = [&](int v) {
        const Vec3f &p = selector.original_vertex(v);
        const bool right_side  = std::abs(p.x() - 15.f) < 1e-3f || std::abs(p.x() - 15.5f) < 1e-3f;
        const bool bottom_side = std::abs(p.y() - 5.f) < 1e-3f || std::abs(p.y() - 4.5f) < 1e-3f;
        return (right_side && p.y() > 4.4f) || (bottom_side && p.x() < 15.6f);
    };
    auto share_on_edge = [&](const std::vector<int> &path) {
        float total = 0.f, good = 0.f;
        for (size_t i = 1; i < path.size(); ++i) {
            const float length = (selector.original_vertex(path[i]) - selector.original_vertex(path[i - 1])).norm();
            total += length;
            if (on_block_edge(path[i]) && on_block_edge(path[i - 1]))
                good += length;
        }
        return total > 0.f ? good / total : 0.f;
    };

    const std::vector<int> snapped = selector.snapped_edge_path(from, to, 20.f);
    REQUIRE(snapped.size() >= 2);
    CHECK(snapped.front() == from);
    CHECK(snapped.back() == to);
    CHECK(share_on_edge(snapped) >= 0.95f);

    const std::vector<int> shortest = selector.snapped_edge_path(from, to, 0.f);
    REQUIRE(shortest.size() >= 2);
    CHECK(share_on_edge(shortest) < 0.6f); // the plain shortest path cuts across the top

    SECTION("no path between unconnected parts") {
        CHECK(selector.snapped_edge_path(from, from, 20.f) == std::vector<int>{from});
        CHECK(selector.snapped_edge_path(from, -1, 20.f).empty());
    }
}

TEST_CASE("Nearest original vertex of a clicked triangle", "[TriangleSelector][BorderLine]")
{
    const TriangleMesh mesh = grid_mesh(3, 1.f);
    TriangleSelector   selector(mesh);
    const auto &tri = mesh.its.indices[0];
    CHECK(selector.nearest_original_vertex(0, mesh.its.vertices[tri[1]] + Vec3f(0.01f, 0.f, 0.f)) == tri[1]);
    CHECK(selector.nearest_original_vertex(-1, Vec3f::Zero()) == -1);
}
