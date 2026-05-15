#pragma once

#include <chrono>
#include <string>
#include <expected>
#include <optional>

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/convert.hpp"
#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

template <typename T>
concept ChronoDuration = requires {
  typename T::rep;
  typename T::period;
  requires std::is_same_v<T, std::chrono::duration<typename T::rep, typename T::period>>;
};

enum class TimedBoxErrors
{
  Expired,
  NotSet
};

/**
 * @brief A box that holds a item for a certain lifespan
 * @tparam ItemT The item type
 */
template <typename ItemT>
class TimedBox
{
public:

  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

  struct ItemContainer
  {
    ItemT item;
    TimePoint set_time;
  };

  /**
 * @brief Construct a new TimedBox with no lifespan
 */
  explicit TimedBox()
  {
  }

  /**
   * @brief Construct a new TimedBox
   * @param lifespan The lifespan for the subscriber before it returns an Expired error
   */
  template <ChronoDuration DurationT>
  explicit TimedBox(
    const DurationT& lifespan)
  {
    set_lifespan(lifespan);
  }

  template <ChronoDuration DurationT>
  void set_lifespan(const DurationT & lifespan)
  {
    lifespan_ = std::chrono::duration_cast<std::chrono::milliseconds>(lifespan);
  }

  void set(const ItemT& item) {
      auto incoming_item = ItemContainer{item, std::chrono::system_clock::now()};
      item_ = incoming_item;
  }

  TimedBox& operator=(const ItemT& item) {
    set(item);
    return *this;
  }

  /**
   * @brief Get the item as an Expected, or reason why it is not available
   * @return The item or an error enum
   */
  std::expected<ItemT, TimedBoxErrors> value() const
  {
    if (!item_) {
      return std::unexpected(TimedBoxErrors::NotSet);
    }
    if (is_expired()) {
      return std::unexpected(TimedBoxErrors::Expired);
    }
    return item_.value().item;
  }

  /**
   * @brief Get the item or a default value if it is not available
   * @param default_value The default value to return if the item is not available
   * @return The item or the default value
   */
  ItemT value_or(const ItemT& default_value) const
  {
    auto val = value();
    if (val) {
      return val.value();
    }
    return default_value;
  }

  /**
   * @brief Get the time the item was last set.
   *
   * @return std::optional<TimePoint> Last set time
   */
  [[nodiscard]] std::optional<TimePoint> get_last_set_time() const
  {
    return item_ ? std::optional<TimePoint>(item_.value().set_time) : std::nullopt;
  }

  /**
   * @brief Check if the item is expired
   * @return True if the item is expired, or has never been set
   */
  [[nodiscard]] bool is_expired() const
  {
    if (!item_) {
      // if the item has never been set, consider it expired
      return true;
    }

    if (!lifespan_.has_value()) {
      // if no lifespan is set, the item should not be considered expired
      return false;
    }

    return is_expired(item_.value());
  }

protected:

  /**
   * @brief Check if the item is expired
   * @param item_container The item container
   * @return True if the item is expired
   */
  [[nodiscard]] bool is_expired(const ItemContainer & item_container) const
  {
    const auto time_since_last_item = std::chrono::system_clock::now() - item_container.set_time;
    return time_since_last_item > lifespan_;
  }

  std::optional<ItemContainer> item_;
  std::optional<std::chrono::milliseconds> lifespan_;
};

inline std::string to_string(TimedBoxErrors e)
{
  switch (e) {
    case TimedBoxErrors::Expired: return "Item lifespan expired";
    case TimedBoxErrors::NotSet:  return "Item has not been set";
    default:         return "Unknown TimedBox error";
  }
}

    inline geometry_msgs::msg::Vector3 transform_enu_to_flu(const geometry_msgs::msg::Vector3& enu_vec, const tf2::Quaternion & flu_orientation)
    {
        geometry_msgs::msg::Vector3 flu_vec;
        // Invert orientation (in FLU) to transform from ENU to FLU
        const tf2::Quaternion q_enu_to_flu = flu_orientation.inverse();
        tf2::Vector3 tmp;
        tf2::fromMsg(enu_vec, tmp);
        tf2::Vector3 out_vec = tf2::quatRotate(q_enu_to_flu, tmp);
        flu_vec = tf2::toMsg(out_vec);
        return flu_vec;
    }

    inline geometry_msgs::msg::Vector3 transform_frd_to_flu(const geometry_msgs::msg::Vector3& frd_vec) {
        // To transform from FRD to FLU, we can rotate 180 degrees around the X axis (roll)
        tf2::Quaternion q_frd_to_flu = tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI);
        tf2::Vector3 tmp;
        tf2::fromMsg(frd_vec, tmp);
        tf2::Vector3 out_vec = tf2::quatRotate(q_frd_to_flu, tmp);
        return tf2::toMsg(out_vec);
    }

    inline geometry_msgs::msg::Vector3 transform_flu_to_frd(const geometry_msgs::msg::Vector3& flu_vec) {
        // same operation
        return transform_frd_to_flu(flu_vec);
    }

    inline geometry_msgs::msg::Vector3 transform_flu_to_enu(const geometry_msgs::msg::Vector3& flu_vec, const tf2::Quaternion & flu_orientation) {
        tf2::Vector3 tmp;
        tf2::fromMsg(flu_vec, tmp);
        tf2::Vector3 out_vec = tf2::quatRotate(flu_orientation, tmp);
        return tf2::toMsg(out_vec);
    }

    inline geometry_msgs::msg::Vector3 transform_frd_to_enu(const geometry_msgs::msg::Vector3& frd_vec, const tf2::Quaternion & frd_orientation) {
        // To transform from FRD to ENU, we can rotate 180 degrees around the X axis (roll)
        tf2::Quaternion q_frd_to_enu = frd_orientation * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI);
        tf2::Vector3 tmp;
        tf2::fromMsg(frd_vec, tmp);
        tf2::Vector3 out_vec = tf2::quatRotate(q_frd_to_enu, tmp);
        return tf2::toMsg(out_vec);
    }

    inline geometry_msgs::msg::Twist transform_enu_to_flu(const geometry_msgs::msg::Twist& enu_twist, const tf2::Quaternion & flu_orientation)
    {
        const tf2::Quaternion q_enu_to_flu = flu_orientation.inverse();
        tf2::Vector3 tmp_vel, tmp_ang;
        tf2::fromMsg(enu_twist.linear, tmp_vel);
        tf2::fromMsg(enu_twist.angular, tmp_ang);
        tf2::Vector3 transformed_velocity = tf2::quatRotate(q_enu_to_flu, tmp_vel);
        tf2::Vector3 transformed_angular = tf2::quatRotate(q_enu_to_flu, tmp_ang);

        geometry_msgs::msg::Twist flu_twist;
        flu_twist.linear = tf2::toMsg(transformed_velocity);
        flu_twist.angular = tf2::toMsg(transformed_angular);
        return flu_twist;
    }

    inline geometry_msgs::msg::Twist transform_frd_to_flu(const geometry_msgs::msg::Twist& frd_twist) {
        geometry_msgs::msg::Twist flu_twist;
        flu_twist.linear = transform_frd_to_flu(frd_twist.linear);
        flu_twist.angular = transform_frd_to_flu(frd_twist.angular);
        return flu_twist;
    }

    inline tf2::Quaternion transform_frd_ned_to_flu_enu(const tf2::Quaternion & q_frd) {
      // NED→ENU world frame change: 180° around the NE-diagonal axis (1/√2, 1/√2, 0)
      const tf2::Quaternion q_ned_to_enu(tf2::Vector3(M_SQRT1_2, M_SQRT1_2, 0.0), M_PI);
      // FRD→FLU body convention change: 180° around X (Forward axis)
      const tf2::Quaternion q_x180(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
      // Left-multiply by world frame change, right-multiply by body convention change
      return q_ned_to_enu * q_frd * q_x180;
    }

