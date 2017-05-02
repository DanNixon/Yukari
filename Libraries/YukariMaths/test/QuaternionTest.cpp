/** @file */

#include <Eigen/Geometry>
#include <boost/test/unit_test.hpp>

static void rotate_point_by_quat_eigen(float qw, float qx, float qy, float qz, float *x, float *y,
                                       float *z)
{
  Eigen::Quaternionf q(qw, qx, qy, qz);
  Eigen::Quaternionf p(0.0f, *x, *y, *z);

  p = q * p;
  p = p * q.conjugate();

  *x = p.x();
  *y = p.y();
  *z = p.z();
}

struct Quat
{
  float w, x, y, z;
};

static Quat quat_mult(Quat a, Quat b)
{
  Quat o;

  o.w = (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z);
  o.x = (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y);
  o.y = (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x);
  o.z = (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w);

  return o;
}

static void rotate_point_by_quat(float qw, float qx, float qy, float qz, float *x, float *y,
                                 float *z)
{
  Quat q = {qw, qx, qy, qz};
  Quat p = {0.0f, *x, *y, *z};

  p = quat_mult(q, p);
  p = quat_mult(p, {q.w, -q.x, -q.y, -q.z});

  *x = p.x;
  *y = p.y;
  *z = p.z;
}

namespace Yukari
{
namespace Maths
{
  namespace Test
  {
    BOOST_AUTO_TEST_SUITE(QuaternionTest)

    BOOST_AUTO_TEST_CASE(rotate_point_by_quat_eigen_Test)
    {
      float x, y, z;

      {
        x = 5.0f;
        y = 3.7f;
        z = -2.1f;
        rotate_point_by_quat_eigen(1.0f, 0.0f, 0.0f, 0.0f, &x, &y, &z);
        BOOST_CHECK_CLOSE(x, 5.0f, 0.1f);
        BOOST_CHECK_CLOSE(y, 3.7f, 0.1f);
        BOOST_CHECK_CLOSE(z, -2.1f, 0.1f);
      }

      {
        x = 1.0f;
        y = 0.0f;
        z = 0.0f;
        rotate_point_by_quat_eigen(0.7071065431725605f, 0.0f, 0.7071070192004544f, 0.0f, &x, &y,
                                   &z);
        BOOST_CHECK(std::abs(x) < 1e-6f);
        BOOST_CHECK(std::abs(y) < 1e-6f);
        BOOST_CHECK_CLOSE(z, -1.0f, 0.1f);
      }

      {
        x = 1.0f;
        y = 0.0f;
        z = 0.0f;
        rotate_point_by_quat_eigen(0.9238794681051637f, 0.0f, 0.0f, 0.38268358785518847f, &x, &y,
                                   &z);
        const float h = std::sqrt(2.0f) / 2.0f;
        BOOST_CHECK_CLOSE(x, h, 0.1f);
        BOOST_CHECK_CLOSE(y, h, 0.1f);
        BOOST_CHECK(std::abs(z) < 1e-6f);
      }
    }

    BOOST_AUTO_TEST_CASE(rotate_point_by_quat_Test)
    {
      struct TestValues
      {
        float qw;
        float qx;
        float qy;
        float qz;
        float x;
        float y;
        float z;
      };

      std::vector<TestValues> testCases;
      testCases.push_back({1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 3.0f});
      testCases.push_back({0.7071065431725605f, 0.0f, 0.0f, 0.7071070192004544f, 1.0f, 2.0f, 3.0f});
      testCases.push_back(
          {0.9238794681051637f, 0.0f, 0.0f, 0.38268358785518847f, 1.0f, 2.0f, 3.0f});
      testCases.push_back(
          {0.991836f, 0.042895f, -0.014131f, -0.104159f, 0.017578f, 0.080933f, 0.910278f});

      for (auto it = testCases.begin(); it != testCases.end(); ++it)
      {
        float x1, y1, z1, x2, y2, z2;

        x1 = it->x;
        x2 = it->x;
        y1 = it->y;
        y2 = it->y;
        z1 = it->z;
        z2 = it->z;

        rotate_point_by_quat(it->qw, it->qx, it->qy, it->qz, &x1, &y1, &z1);
        rotate_point_by_quat_eigen(it->qw, it->qx, it->qy, it->qz, &x2, &y2, &z2);

        BOOST_CHECK_CLOSE(x1, x2, 0.1f);
        BOOST_CHECK_CLOSE(y1, y2, 0.1f);
        BOOST_CHECK_CLOSE(z1, z2, 0.1f);
      }
    }

    BOOST_AUTO_TEST_SUITE_END()
  }
}
}
