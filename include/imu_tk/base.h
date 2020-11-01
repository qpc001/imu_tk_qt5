/* 
 * imu_tk - Inertial Measurement Unit Toolkit
 *
 *  Copyright (c) 2014, Alberto Pretto <pretto@diag.uniroma1.it>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Core>

namespace imu_tk
{
/** @brief Simple container for a data item (e.g., timestamp + x, y, z accelerometers or
 *         gyroscopes reading
 *  @brief 简单容器， 时间戳,x,y,z 加速度或角速度
 */
template <typename _T > class TriadData_
{
public:
    //////////////////
    /// 一堆构造函数
    //////////////////
    /** @brief Construct an uninitialized TriadData_ object */
    TriadData_() {};

    /** @brief Construct a TriadData_ object from a timestamp and three values */
    TriadData_ ( _T timestamp, _T x, _T y, _T z ) :
        timestamp_ ( timestamp ),
        data_ ( x, y, z ) {};

    /** @brief Construct an TriadData_ object from a timestamp and an Eigen vector */
    TriadData_ ( _T timestamp, const Eigen::Matrix< _T, 3, 1> &data ) :
        timestamp_ ( timestamp ),
        data_ ( data ) {};

    /** @brief Construct an TriadData_ object from a timestamp and an array with 3 elements */
    TriadData_ ( _T timestamp, const _T *data ) :
        timestamp_ ( timestamp ),
        data_ ( data[0], data[1], data[2] ) {};

    /** @brief Copy constructor (拷贝构造函数)*/
    TriadData_( const TriadData_ &o ) :
        timestamp_(o.timestamp_), data_(o.data_) {};


    /** @brief Copy assignment operator (拷贝赋值运算符) */
    TriadData_ & operator = (const TriadData_ &o )
    {
        timestamp_ = o.timestamp_;
        data_ = o.data_;
        return *this;
    };

    /** @brief  Copy constructor  拷贝构造函数(支持模板类构造)
     *
     * Supporting coercion using member template constructor */
    template< typename _newT >
    TriadData_( const TriadData_<_newT> &o ) :
        timestamp_(_T(o.timestamp())), data_(o.data().template cast<_T>())
    {};

    /** @brief Copy assignment operator 拷贝赋值运算符(支持模板类构造)
   *
   * Supporting coercion using member template assignment operator */
    template< typename _newT >
    TriadData_ & operator = (const TriadData_<_newT> &o )
    {
        timestamp_ = _T(o.timestamp());
        data_ = o.data().template cast<_T>();
        return *this;
    };
    
    ~TriadData_() {};

    inline const _T& timestamp() const
    {
        return timestamp_;
    };
    // 返回data_矩阵
    inline const Eigen::Matrix< _T, 3, 1>& data() const
    {
        return data_;
    };
    // 运算符重载
    inline const _T& operator() ( int index ) const
    {
        return data_[index];
    };
    inline const _T& x() const
    {
        return data_[0];
    };
    inline const _T& y() const
    {
        return data_[1];
    };
    inline const _T& z() const
    {
        return data_[2];
    };

private:
    Eigen::Matrix< _T, 3, 1> data_;
    _T timestamp_;
};

typedef TriadData_<double> TriadData;

/**
 * @brief Generates a sequence of characters with a properly formatted
 *         representation of a TriadData_  instance (triad_data),
 *         and inserts them into the output stream os.
 * @brief 运算符"<<"重载，使用特定格式的TriadData_生成序列字符，用来输出，用于保存
 *        具体实现在下面
 */
template <typename _T> 
std::ostream& operator<<(std::ostream& os, const TriadData_<_T>& triad_data);


/**
 * @brief Simple structure that defines a data vector interval by means of an initial and
 *         and a final index.
 * @brief 使用 初始和结束 idx来定义 某个时间断内的数据
 */
struct DataInterval
{
    /**
     * @brief Default constructor (without parameters, it constructs an invalid interval,
     *         i.e. both -1 indices )
     */
    DataInterval ( int start_idx = -1, int end_idx = -1 ) :
        start_idx ( start_idx ), end_idx ( end_idx ) {};

    /** @brief Provides a DataInterval object from a time interval. The indices are extracted
     *         from the data samples vector using a nearest neighbor approach.
     *  @brief 从数据vector中使用最近临搜索，获取给定时间戳间隔内的数据
     *
     * @param samples Input signal (data samples vector)
     * @param start_ts Initial timestamp
     * @param end_ts Final timestamp
     */
    template <typename _T>
    static DataInterval fromTimestamps( const std::vector< TriadData_<_T> > &samples,
                                        _T start_ts, _T end_ts )
    {
        if( start_ts < 0 || end_ts <= start_ts )
            throw std::invalid_argument("Invalid timestamps");
        if( samples.size() < 3 )
            throw std::invalid_argument("Invalid data samples vector");

        int start_idx, end_idx;
        if( start_ts <= samples[0].timestamp() )
            start_idx = 0;
        else
            start_idx = timeToIndex( samples, start_ts );

        if( end_ts >= samples[samples.size() - 1].timestamp() )
            end_idx = samples.size() - 1;
        else
            end_idx = timeToIndex( samples, end_ts );

        return DataInterval( start_idx, end_idx );
    };

    /**
     * @brief Extracts from the data samples vector a DataInterval object that represents
     *         the initial interval with a given duration.
     * @brief 从给定数据vector中，以第一个数据为起始，获取给定时间间隔内的数据
     *
     * @param samples Input signal (data samples vector)
     * @param duration Interval duration
     */
    template <typename _T>
    static DataInterval initialInterval( const std::vector< TriadData_<_T> > &samples,
                                         _T duration )
    {
        if( duration <= 0)
            throw std::invalid_argument("Invalid interval duration");
        if( samples.size() < 3 )
            throw std::invalid_argument("Invalid data samples vector");

        _T end_ts = samples[0].timestamp() + duration;
        int end_idx;
        if ( end_ts >=  samples[samples.size() - 1].timestamp() )
            end_idx = samples.size() - 1;
        else
            end_idx = timeToIndex( samples, end_ts );

        return DataInterval( 0, end_idx );
    };

    /**
     * @brief Extracts from the data samples vector a DataInterval object that represents
     *         the final interval with a given duration.
     * @brief 从给定数据vector中，以最后一个数据为结束，获取最后duration内的时间戳
     *
     * @param samples Input signal (data samples vector)
     * @param duration Interval duration
     */
    template <typename _T>
    static DataInterval finalInterval( const std::vector< TriadData_<_T> > &samples,
                                       _T duration )
    {
        if( duration <= 0)
            throw std::invalid_argument("Invalid interval duration");
        if( samples.size() < 3 )
            throw std::invalid_argument("Invalid data samples vector");

        _T start_ts = samples[samples.size() - 1].timestamp() - duration;
        int start_idx;
        if ( start_ts <= 0 )
            start_idx = 0;
        else
            start_idx = timeToIndex( samples, start_ts );

        return DataInterval( start_idx, samples.size() - 1 );
    };
    
    int start_idx, end_idx;

private:
    /**
     * @brief 取数据队列中时间上最接近ts的idx
     */
    template <typename _T> static int timeToIndex( const std::vector< TriadData_<_T> > &samples,
                                                   _T ts )
    {
        int idx0 = 0, idx1 = samples.size() - 1, idxm;
        while( idx1 - idx0 > 1 )
        {
            idxm = (idx1 + idx0)/2;
            if( ts > samples[idxm].timestamp() )
                idx0 = idxm;
            else
                idx1 = idxm;
        }

        if( ts - samples[idx0].timestamp() < samples[idx1].timestamp() - ts )
            return idx0;
        else
            return idx1;
    };
};

/**
 * @brief Perform a simple consistency check on a target input interval,
 *         given an input data sample vector, and return the "corrected"
 *         interval
 * @brief 检查数据队列中，给定时间间隔内的数据是否合法
 *
 * @param samples Input signal (data samples vector)
 * @param interval Interval to be checked
 *
 * @returns The "corrected" interval
 */
template <typename _T> 
DataInterval checkInterval( const std::vector< TriadData_<_T> > &samples,
                            const DataInterval &interval );

/**
 * @brief Compute the arithmetic mean of a sequence of TriadData_ objects. If a valid data
 *         interval is provided, the mean is computed only inside this interval
 * @brief 计算给定数据队列，指定时间间隔内的数据均值，如果给定范围不合法，则计算全部数据的均值
 *
 * @param samples Input signal (data samples vector)
 * @param interval Data interval where to compute the mean. If this interval is not valid,
 *                 i.e., one of the two indices is -1, the mean is computed for the whole data
 *                 sequence.
 *
 * @returns A three dimensional vector representing the mean.
 */
template <typename _T> 
Eigen::Matrix< _T, 3, 1> dataMean ( const std::vector< TriadData_<_T> > &samples,
                                    const DataInterval &interval = DataInterval() );

/**
 * @brief Compute the variance of a sequence of TriadData_ objects. If a valid data
 *         interval is provided, the variance is computed only inside this interval
 * @brief 计算给定数据队列，指定时间间隔内的数据方差，如果给定范围不合法，则计算全部数据的方差
 *
 * @param samples Input signal (data samples vector)
 * @param interval Data interval where to compute the variance. If this interval is not valid,
 *                 i.e., one of the two indices is -1, the variance is computed for the whole data
 *                 sequence.
 *
 * @returns A three dimensional vector representing the variance.
 */
template <typename _T>
Eigen::Matrix< _T, 3, 1> dataVariance ( const std::vector< TriadData_<_T> > &samples,
                                        const DataInterval &interval = DataInterval() );

/** @brief If the flag only_means is set to false, for each interval 
  *        (input vector intervals) extract from the input signal
  *        (samples) the first interval_n_samps samples, and store them
  *        in the output vector extracted_samples. If the flag only_means
  *        is set to true, extract for each interval only the local mean, computed
  *        in interval with size at least interval_n_samps samples.
  *        Only intervals with at least interval_n_samps samples are considered.
  * @brief 如果 'only_means' 标志为flase，则从总数据队列中，提取每一个间隔的前n个采样结果，然后储存到extracted_samples[i]
  *        如果 'only_means' 标志为true，则只提取均值
  *        以上，只有当间隔内的数据大于n，这个间隔才会被考虑
  *
  * @param samples Input signal (data samples vector)
  * @param intervals Input intervals vector
  * @param[out] extracted_samples Output signal that contains the extracted samples
  * @param[out] extracted_intervals Output intervals vector with all the used intervals, i.e.
  *                                 intervals with size at least interval_n_samps samples
  * @param interval_n_samps Number of samples to be extracted from each interval (or
  *                         interval size to be used to compute the local mean if
  *                         only_means is set to true)
  * @param only_means If true, extract for each interval only the local mean, computed
  *                   in intervals with size at least interval_n_samps samples. The timestamp is
  *                   the one of the center of the interval.
  *
  */
template <typename _T> 
void extractIntervalsSamples ( const std::vector< TriadData_<_T> > &samples,
                               const std::vector< DataInterval > &intervals,
                               std::vector< TriadData_<_T> > &extracted_samples,
                               std::vector< DataInterval > &extracted_intervals,
                               int interval_n_samps = 100, bool only_means = false );

/** @brief Decompose a rotation matrix into the roll, pitch, and yaw angular components
 *  @brief 从旋转矩阵中分解roll pitch yaw
 *
 *  @param rot_mat Input rotation matrix
 *  @param[out] rpy_rot_vec Output roll, pitch, and yaw angular components
 */
template <typename _T> void decomposeRotation( const Eigen::Matrix< _T, 3, 3> &rot_mat,
                                               Eigen::Matrix< _T, 3, 1> &rpy_rot_vec );

/* Implementations */

template <typename _T>
std::ostream& operator<<(std::ostream& os, const TriadData_<_T>& triad_data)
{
    os<<"ts : "<<triad_data.timestamp();
    os<<" data : [ ";
    os<<triad_data.x()<<", ";
    os<<triad_data.y()<<", ";
    os<<triad_data.z()<<" ]";

    return os;
}

template <typename _T>
DataInterval checkInterval( const std::vector< TriadData_<_T> > &samples,
                            const DataInterval &interval )
{
    int start_idx = interval.start_idx, end_idx = interval.end_idx;
    if( start_idx < 0) start_idx = 0;
    if( end_idx < start_idx || end_idx > samples.size() - 1 )
        end_idx = samples.size() - 1;

    return DataInterval( start_idx, end_idx );
}

template <typename _T>
Eigen::Matrix< _T, 3, 1> dataMean( const std::vector< TriadData_<_T> >& samples,
                                   const DataInterval& interval )
{
    DataInterval rev_interval =  checkInterval( samples, interval );
    int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
    Eigen::Matrix< _T, 3, 1> mean(0, 0, 0);

    for( int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++)
        mean += samples[i].data();

    mean /= _T(n_samp);

    return mean;
}

template <typename _T>
Eigen::Matrix< _T, 3, 1> dataVariance( const std::vector< TriadData_<_T> >& samples,
                                       const DataInterval& interval )
{
    DataInterval rev_interval =  checkInterval( samples, interval );
    int n_samp = rev_interval.end_idx - rev_interval.start_idx + 1;
    Eigen::Matrix< _T, 3, 1> mean = dataMean( samples, rev_interval );

    Eigen::Matrix< _T, 3, 1> variance(0, 0, 0);
    for( int i = rev_interval.start_idx; i <= rev_interval.end_idx; i++)
    {
        Eigen::Matrix< _T, 3, 1> diff = samples[i].data() - mean ;
        variance += (diff.array() * diff.array()).matrix();
    }
    variance /= _T(n_samp - 1);

    return variance;
}

template <typename _T>
void extractIntervalsSamples ( const std::vector< TriadData_<_T> >& samples,
                               const std::vector< DataInterval >& intervals,
                               std::vector< TriadData_<_T> >& extracted_samples,
                               std::vector< DataInterval > &extracted_intervals,
                               int interval_n_samps, bool only_means )
{
    // Check for valid intervals  (i.e., intervals with at least interval_n_samps samples)
    // 检查有效的间隔，间隔所包含的采样次数应>n
    int n_valid_intervals = 0, n_static_samples;
    for( int i = 0; i < intervals.size(); i++)
    {
        if( ( intervals[i].end_idx - intervals[i].start_idx + 1 ) >= interval_n_samps )
            n_valid_intervals++;
    }
    // n_valid_intervals: 提取到的有效的静态间隔
    // n_static_samples: 提取到的有效的所有静态间隔的所有数据量之和
    if( only_means )
        n_static_samples = n_valid_intervals;
    else
        n_static_samples = n_valid_intervals*interval_n_samps;

    extracted_samples.clear();
    extracted_intervals.clear();
    extracted_samples.reserve(n_static_samples);
    extracted_intervals.reserve(n_valid_intervals);

    // For each valid interval, extract the first interval_n_samps samples
    // 对于每一个有效的间隔，提取该间隔内前n个采样数据
    for( int i = 0; i < intervals.size(); i++)
    {
        int interval_size = intervals[i].end_idx - intervals[i].start_idx + 1;
        if( interval_size >= interval_n_samps )
        {
            // extracted_intervals：储存每一个有效间隔
            extracted_intervals.push_back( intervals[i] );
            if( only_means )
            {
                DataInterval mean_inerval( intervals[i].start_idx, intervals[i].end_idx );
                // Take the timestamp centered in the interval where the mean is computed
                _T timestamp = samples[ intervals[i].start_idx + interval_size/2 ].timestamp();
                Eigen::Matrix< _T, 3, 1> mean_val = dataMean ( samples, mean_inerval );
                extracted_samples.push_back( TriadData_<_T>(timestamp, mean_val ) );
            }
            else
            {
                for(int j = intervals[i].start_idx; j < intervals[i].start_idx + interval_n_samps; j++)
                    extracted_samples.push_back( samples[j] );
            }
        }
    }
}

template <typename _T> void decomposeRotation( const Eigen::Matrix< _T, 3, 3> &rot_mat,
                                               Eigen::Matrix< _T, 3, 1> &rpy_rot_vec )
{
    rpy_rot_vec(0) = atan2(rot_mat(2,1), rot_mat(2,2));
    rpy_rot_vec(1) = atan2(-rot_mat(2,0), sqrt(rot_mat(2,1)*rot_mat(2,1) + rot_mat(2,2)*rot_mat(2,2)));
    rpy_rot_vec(2) = atan2(rot_mat(1,0), rot_mat(0,0));
}

}
