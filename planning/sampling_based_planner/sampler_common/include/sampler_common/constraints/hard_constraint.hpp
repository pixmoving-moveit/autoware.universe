// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLER_COMMON__CONSTRAINTS__HARD_CONSTRAINT_HPP_
#define SAMPLER_COMMON__CONSTRAINTS__HARD_CONSTRAINT_HPP_

#include "sampler_common/structures.hpp"

#include <vector>
namespace sampler_common::constraints
{
/// @brief Check if the path satisfies the hard constraints
MultiPoint2d checkHardConstraints(Path & path, const Constraints & constraints);
bool has_collision(const MultiPoint2d & footprint, const MultiPolygon2d & obstacles);
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max);

}  // namespace sampler_common::constraints

#endif  // SAMPLER_COMMON__CONSTRAINTS__HARD_CONSTRAINT_HPP_
