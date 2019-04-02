

/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TABLE_CONTROLLER_H
#define TABLE_CONTROLLER_H

#include <fstream>

class Table
{
private:
  // table[speed_index][accleration_index] = command
  std::vector<std::vector<double>> table_;
  const double CMIN = 0.0, CMAX = 1.0;
  const double smin_, smax_, ds_;
  const double amin_, amax_, da_;
  bool validS(const double& s)
  {
    return (smin_ <= s && s <= smax_);
  }
  bool validA(const double& a)
  {
    return (amin_ <= a && a <= amax_);
  }
  bool validC(const double& c)
  {
    return (CMIN <= c && c <= CMAX);
  }
  unsigned int getSI(const double& s)
  {
    return (unsigned int)(std::fabs(s - smin_) / ds_);
  }
  unsigned int getAI(const double& a)
  {
    return (unsigned int)(std::fabs(a - amin_) / da_);
  }

public:
  Table(const double& smin, const double& smax, const double& ds, const double& amin, const double& amax,
        const double& da)
    : smin_(smin), smax_(smax), ds_(ds), amin_(amin), amax_(amax), da_(da)
  {
    table_.resize(getSI(smax) + 1);
    for (auto& ts : table_)
    {
      ts.resize(getAI(amax) + 1);
      for (auto& ta : ts)
      {
        ta = -1.0;
      }
    }
  }

  bool setCommand(const double& s, const double& a, const double& c)
  {
    if (validS(s) && validA(a) && validC(c))
    {
      table_[getSI(s)][getAI(a)] = c;
      return true;
    }
    else
    {
      return false;
    }
  }
  bool getCommand(const double& s, const double& a, double& c)
  {
    if (validS(s) && validA(a))
    {
      double x = table_[getSI(s)][getAI(a)];
      if (validC(x))
      {
        c = x;
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
};

class TableController
{
private:
  std::shared_ptr<Table> th_table_, br_table_;
public:
  TableController()
  {
  }

  bool loadTable(const std::string& path, std::shared_ptr<Table>& table, const double& ds = 0.2, const double& da = 0.1)
  {
    std::ifstream ifs(path);
    if (!ifs)
    {
      return false;
    }

    // command, speed, acceleration
    std::vector<std::vector<double>> csa(3);
    std::string buf;
    for (size_t row = 0; getline(ifs, buf); ++row)
    {
      if (row == 0)
      {
        continue;
      }
      std::string token;
      std::istringstream stream(buf);
      for (size_t col = 0; getline(stream, token, ','); ++col)
      {
        csa[col].push_back(std::stof(token));
      }
    }

    if (csa[0].size() != csa[1].size() || csa[0].size() != csa[2].size())
    {
      return false;
    }

    double smin = *std::min_element(csa[1].begin(), csa[1].end());
    double smax = *std::max_element(csa[1].begin(), csa[1].end());
    double amin = *std::min_element(csa[2].begin(), csa[2].end());
    double amax = *std::max_element(csa[2].begin(), csa[2].end());

    table = std::shared_ptr<Table>(new Table(smin, smax, ds, amin, amax, da));

    for (size_t col = 0; col < csa[0].size(); col++)
    {
      table->setCommand(csa[1][col], csa[2][col], csa[0][col]);
    }

    return true;
  }

  bool loadTables(const std::string& th_path, const std::string& br_path)
  {
    if (!loadTable(th_path, th_table_) || !loadTable(br_path, br_table_))
    {
      return false;
    }
  }

  bool getThrottle(const double& speed, const double& acceleration, double& throttle)
  {
    return th_table_->getCommand(speed, acceleration, throttle);
  }

  bool getBrake(const double& speed, const double& acceleration, double& brake)
  {
    return br_table_->getCommand(speed, acceleration, brake);
  }
};

#endif  // TABLE_CONTROLLER_H
