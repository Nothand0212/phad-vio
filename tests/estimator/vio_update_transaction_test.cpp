#include "phad/estimator/internal/vio_update_transaction.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

TEST( VioUpdateTransaction, InPlaceInstallPreservesOwnerIdentity )
{
  using phad::estimator::internal::installVioUpdateStateInPlace;
  using phad::estimator::internal::VioUpdateState;

  auto owner                         = std::make_unique<VioUpdateState>();
  owner->m_segment_id                = 3U;
  VioUpdateState* const owner_before = owner.get();

  VioUpdateState validated;
  validated.m_segment_id = 9U;
  installVioUpdateStateInPlace( owner, std::move( validated ) );

  EXPECT_EQ( owner.get(), owner_before );
  EXPECT_EQ( owner->m_segment_id, 9U );
}

TEST( VioUpdateTransaction, WindowFrameNavigationIsExplicitlyOptional )
{
  using phad::estimator::internal::NavigationState;
  using phad::estimator::internal::WindowFrame;

  WindowFrame frame;
  EXPECT_FALSE( frame.m_navigation.has_value() );

  NavigationState navigation;
  navigation.m_T_W_B.translation() = Eigen::Vector3d( 1.0, 2.0, 3.0 );
  frame.m_navigation               = navigation;

  ASSERT_TRUE( frame.m_navigation.has_value() );
  EXPECT_EQ( frame.m_navigation->m_T_W_B.translation(),
             Eigen::Vector3d( 1.0, 2.0, 3.0 ) );
}

TEST( VioUpdateTransaction, PreparedOwnerInstallIsNoexceptSwap )
{
  using phad::estimator::internal::installVioUpdateStateOwner;
  using phad::estimator::internal::VioUpdateState;

  auto live                       = std::make_unique<VioUpdateState>();
  live->m_segment_id              = 3U;
  VioUpdateState* const old_owner = live.get();

  auto validated                        = std::make_unique<VioUpdateState>();
  validated->m_segment_id               = 9U;
  VioUpdateState* const validated_owner = validated.get();

  static_assert( noexcept( installVioUpdateStateOwner( live, validated ) ) );
  installVioUpdateStateOwner( live, validated );

  EXPECT_EQ( live.get(), validated_owner );
  EXPECT_EQ( live->m_segment_id, 9U );
  EXPECT_EQ( validated.get(), old_owner );
  EXPECT_EQ( validated->m_segment_id, 3U );
}
