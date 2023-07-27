/**
 * @file filters.h
 *
 * Created : 31 May, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#include <math.h>
#include <vector>

#include "utils/transformations.h"

namespace filters
{

  template<typename T>
  T giveMedian(utils::VectorX<T> a)
  {
    assert(a.size() != 0 && "Invalid! giveMedian input vector has size = 0");
    assert(a.hasNaN() != true && "Invalid! giveMedian input vector has NaN element");

    // First we sort the array
    int n = a.size();
    std::vector<T> temp(a.data(), a.data() + n);

    std::sort(temp.begin(), temp.end());

    // check for even case
    if (n % 2 != 0)
      return (T)temp[n / 2];

    return (T)(temp[(n - 1) / 2] + temp[n / 2]) / 2.0;
  }

  /**
   * \brief A Class that contains past data used by butterworth filter
   */
  class ImuPastFilterValues
  {
  public:
    utils::VectorX<double> w0_;   // Stores the current value of the calculated data
    utils::VectorX<double> w1_;   // Stores 1 time step older value
    utils::VectorX<double> w2_;   // Stores 2 time step older value
    utils::VectorX<double> w3_;   // Stores 3 time step older value
    utils::VectorX<double> w4_;   // Stores 4 time step older value

    ImuPastFilterValues(double filter_order)
    {
      w0_.resize(filter_order);
      w1_.resize(filter_order);
      w2_.resize(filter_order);
      w3_.resize(filter_order);
      w4_.resize(filter_order);
    }

    ImuPastFilterValues()
    {
      w0_.resize(4);
      w1_.resize(4);
      w2_.resize(4);
      w3_.resize(4);
      w4_.resize(4);
    }

    void resizeArray(double filter_order)
    {
      w0_.resize(filter_order);
      w1_.resize(filter_order);
      w2_.resize(filter_order);
      w3_.resize(filter_order);
      w4_.resize(filter_order);
    }
  };

  /**
   * \brief A Class that contains functions and variables to perform different filtering operations
   */
  class Filter
  {
  public:
    double filter_order_;        // Filter order we want NOTE: needs to be a multiple of 4
    double sampling_freq_;       // The rate at which we recive the data
    double upper_cutoff_freq_;   // Upper 3-Db cutoff frequency 
    double lower_cutoff_freq_;   // Lower 3-Db cutoff frequency

    utils::VectorX<double> A_;  // Scaling factor to maintain magnitude
    utils::VectorX<double> d1_; // Coefficient 1
    utils::VectorX<double> d2_; // Coefficient 2
    utils::VectorX<double> d3_; // Coefficient 3
    utils::VectorX<double> d4_; // Coefficient 4

    /**
    * \brief Constructor to initialize the features of the filter going to be used
    */
    Filter()
    {
      // Setting these values as default
      filter_order_ = 16;
      sampling_freq_ = 200;
      upper_cutoff_freq_ = 2.3;
      lower_cutoff_freq_ = 2.7;
    }

    /**
    * \brief A function that performs averaging filtering
    *
    * \param[in] avg: The Global variable which is passed to function with past values
    * \param[in] current: The current data that is recived
    */
    double lowPassFilter(utils::VectorX<double>& avg, double current)
    {
      assert(avg.size() != 0 && "Invalid! lowPassFilter input avg vector has size = 0");
      assert(avg.hasNaN() != true && "Invalid! lowPassFilter input avg vector has NaN element");

      double filtered_value;
      avg = utils::shiftByRows(avg, 1);
      avg[avg.size() - 1] = current;

      filtered_value = avg.sum() / avg.size();

      return filtered_value;
    }

    /**
    * \brief A function to find the coefficient of the butterworth filter based on t
    *        filter_order_, sampling_freq_, upper 3-Db cutoff, lower 3-Db cutoff
    *
    * \param[in] filter_order: The filter order we require needs to be a multiple of 4
    * \param[in] sampling_freq: The frequency at which data is recived
    * \param[in] upper_cutoff_freq: The upper 3-Db cutoff
    * \param[in] lower_cutoff_freq: The lower 3-Db cutoff
    * \param[in] A:  Scaling factor to maintain magnitude
    * \param[in] d1: Coefficient 1
    * \param[in] d2: Coefficient 2
    * \param[in] d3: Coefficient 3
    * \param[in] d4: Coefficient 4
    *
    * \param[out] filter_order: This variable is returned as 1/4th of the value is used for the butterworth filter
    */
    double bandPassFilterCoeff(double filter_order, double sampling_freq, double upper_cutoff_freq, double lower_cutoff_freq,
      utils::VectorX<double>& A, utils::VectorX<double>& d1, utils::VectorX<double>& d2, utils::VectorX<double>& d3, utils::VectorX<double>& d4)
    {

      double a = cos(M_PI * (upper_cutoff_freq + lower_cutoff_freq) / sampling_freq) / cos(M_PI * (upper_cutoff_freq - lower_cutoff_freq) / sampling_freq);
      double a2 = a * a;
      double b = tan(M_PI * (upper_cutoff_freq - lower_cutoff_freq) / sampling_freq);
      double b2 = b * b;

      assert((int)filter_order % 4 == 0 && "Invalid! bandPassFilterCoeff has input filter order which is not a multiple of 4");
      filter_order = filter_order / 4;

      A.resize(filter_order);
      d1.resize(filter_order);
      d2.resize(filter_order);
      d3.resize(filter_order);
      d4.resize(filter_order);

      utils::VectorX<double> fil_linspace; fil_linspace.resize(A.size());
      fil_linspace.setLinSpaced(0, filter_order, A.size());
      auto r = (M_PI * (2 * fil_linspace.array() + 1) / (4 * filter_order)).sin();
      auto s = b2 + 2 * b * r.array() + 1;

      A = b2 / s.array();
      d1 = 4 * a * (1 + b * r.array()) / s.array();
      d2 = 2 * (b2 - 2 * a2 - 1) / s.array();
      d3 = 4 * a * (1 - b * r.array()) / s.array();
      d4 = -(b2 - 2 * b * r.array() + 1) / s.array();

      assert(A.hasNaN() != true && "Invalid! bandPassFilterCoeff output A vector has NaN element");
      assert(d1.hasNaN() != true && "Invalid! bandPassFilterCoeff output d1 vector has NaN element");
      assert(d2.hasNaN() != true && "Invalid! bandPassFilterCoeff output d2 vector has NaN element");
      assert(d3.hasNaN() != true && "Invalid! bandPassFilterCoeff output d3 vector has NaN element");
      assert(d4.hasNaN() != true && "Invalid! bandPassFilterCoeff output d4 vector has NaN element");

      return filter_order;
    }

    /**
    * \brief A function to find the coefficient of the butterworth filter based on t
    *        filter_order_, sampling_freq_, upper 3-Db cutoff, lower 3-Db cutoff
    *
    * \param[in] data: The current recived value
    * \param[in] filter_order: The filter order we require needs to be a multiple of 4
    * \param[in] A:  Scaling factor to maintain magnitude
    * \param[in] d1: Coefficient 1
    * \param[in] d2: Coefficient 2
    * \param[in] d3: Coefficient 3
    * \param[in] d4: Coefficient 4
    * \param[in] past_values: Class variable that stores the past values of the sequence of data
    *
    * \param[out] output: The filtered output
    */
    double bandPassFilter(double data, double filter_order,
      utils::VectorX<double>& A, utils::VectorX<double>& d1, utils::VectorX<double>& d2, utils::VectorX<double>& d3, utils::VectorX<double>& d4,
      ImuPastFilterValues& past_values)
    {
      double output = data;
      assert((int)filter_order % 4 == 0 && "Invalid! bandPassFilter has input filter order which is not a multiple of 4");

      assert(A.size() == filter_order && "Invalid! bandPassFilter input A vector has size = 0");
      assert(d1.size() == filter_order && "Invalid! bandPassFilter input d1 vector has size = 0");
      assert(d2.size() == filter_order && "Invalid! bandPassFilter input d2 vector has size = 0");
      assert(d3.size() == filter_order && "Invalid! bandPassFilter input d3 vector has size = 0");
      assert(d4.size() == filter_order && "Invalid! bandPassFilter input d4 vector has size = 0");

      assert(A.hasNaN() != true && "Invalid! bandPassFilter input A vector has NaN element");
      assert(d1.hasNaN() != true && "Invalid! bandPassFilter input d1 vector has NaN element");
      assert(d2.hasNaN() != true && "Invalid! bandPassFilter input d2 vector has NaN element");
      assert(d3.hasNaN() != true && "Invalid! bandPassFilter input d3 vector has NaN element");
      assert(d4.hasNaN() != true && "Invalid! bandPassFilter input d4 vector has NaN element");

      assert(past_values.w0_.size() != 0 && "Invalid! bandPassFilter output w0 vector has size = 0");
      assert(past_values.w1_.size() != 0 && "Invalid! bandPassFilter output w1 vector has size = 0");
      assert(past_values.w2_.size() != 0 && "Invalid! bandPassFilter output w2 vector has size = 0");
      assert(past_values.w3_.size() != 0 && "Invalid! bandPassFilter output w3 vector has size = 0");
      assert(past_values.w4_.size() != 0 && "Invalid! bandPassFilter output w4 vector has size = 0");

      for (int j = 0; j < filter_order_; j++)
      {
        past_values.w0_[j] = d1[j] * past_values.w1_[j] + d2[j] * past_values.w2_[j] + d3[j] * past_values.w3_[j] + d4[j] * past_values.w4_[j] + output;
        output = A[j] * (past_values.w0_[j] - 2 * past_values.w2_[j] + past_values.w4_[j]);

        past_values.w4_[j] = past_values.w3_[j];
        past_values.w3_[j] = past_values.w2_[j];
        past_values.w2_[j] = past_values.w1_[j];
        past_values.w1_[j] = past_values.w0_[j];
      }

      assert(past_values.w0_.hasNaN() != true && "Invalid! bandPassFilter input w0 vector has NaN element");
      assert(past_values.w1_.hasNaN() != true && "Invalid! bandPassFilter input w1 vector has NaN element");
      assert(past_values.w2_.hasNaN() != true && "Invalid! bandPassFilter input w2 vector has NaN element");
      assert(past_values.w3_.hasNaN() != true && "Invalid! bandPassFilter input w3 vector has NaN element");
      assert(past_values.w4_.hasNaN() != true && "Invalid! bandPassFilter input w4 vector has NaN element");

      return output;
    }
  };
}