#ifndef SPACE_TIME_POINT_H
#define SPACE_TIME_POINT_H

#include <guidance_planner/config.h>

#include <Eigen/Dense>

#include <type_traits>

namespace GuidancePlanner
{
    // T states with P positions
    template <int T, int P>
    struct SpaceTimePointDim
    {
        typedef Eigen::Matrix<double, T + 1, 1> Vector;
        typedef Eigen::Matrix<double, T, 1> TVector;
        typedef Eigen::Matrix<double, P, 1> PVector;

        typedef Eigen::Matrix<double, P + 1, 1> PosTimeVector;

    private:
        Vector _vec; // Last element is time [s]

    public:
        SpaceTimePointDim(){};

        SpaceTimePointDim(const Vector &vector) { _vec = vector; } // From vector

        SpaceTimePointDim(const TVector &state_vector, int time)
        {
            for (int i = 0; i < T; i++)
                _vec(i) = state_vector(i);

            _vec(T) = time;
        }

        // Variable number of doubles
        template <typename... Args>
        SpaceTimePointDim(double first, Args... args)
        {
            static_assert(sizeof...(args) == T, "Constructor should contain a value for each dimension");

            _vec(0) = first;
            int index = 1;
            double temp[] = {(_vec(index++) = args)...};
            (void)temp;
        }

        SpaceTimePointDim(const SpaceTimePointDim &other)
        {
            *this = other;
        } // Copy

    public:
        SpaceTimePointDim &operator=(const SpaceTimePointDim &other) // Set
        {
            _vec = static_cast<Vector>(other);
            return *this;
        }

        operator Vector() const { return _vec; } // Cast to Vector

        TVector State() const { return _vec.block(0, 0, T, 1); }
        void SetState(const TVector &val) const { _vec.block(0, 0, T, 1) = val; }

        PosTimeVector PosTime() const
        {
            PosTimeVector result = _vec.block(0, 0, P + 1, 1); // Retrieve Position + 1 State or Time
            result(P) = _vec(T);                               // Set Time to be the last
            return result;
        }

        PVector Pos() const { return _vec.block(0, 0, P, 1); }
        void SetPos(const PVector &val) { _vec.block(0, 0, P, 1) = val; }

        double &Time() { return _vec(T); }
        double Time() const { return _vec(T); }
        void SetTime(double time) { _vec(T) = time; }

        double Norm() const { return _vec.norm(); }
        void Normalize() { _vec.normalize(); }

        double &operator()(int i) { return _vec(i); }

        static int numStates() { return T; }
        static int numPositions() { return P; }

        SpaceTimePointDim operator+(const SpaceTimePointDim &other) const
        {
            return SpaceTimePointDim(_vec + static_cast<Vector>(other));
        }

        SpaceTimePointDim operator-(const SpaceTimePointDim &other) const
        {
            return SpaceTimePointDim(_vec - static_cast<Vector>(other));
        }

        SpaceTimePointDim operator*(const double &other) const
        {
            return SpaceTimePointDim(static_cast<Vector>(other) * _vec);
        }

        friend SpaceTimePointDim operator*(const double &first, const SpaceTimePointDim &other)
        {
            return SpaceTimePointDim(first * static_cast<Vector>(other));
        }

        // Const version (scale this vectors time axis with DT), i.e., map to time in seconds
        PosTimeVector MapToTime() const
        {
            PosTimeVector copied_vec = PosTime();
            copied_vec(P) *= Config::DT;
            return copied_vec;
        }

        friend std::ostream &operator<<(std::ostream &stream, const SpaceTimePointDim &point)
        {
            auto vec = static_cast<Vector>(point);
            for (int i = 0; i < T + 1; i++)
            {
                stream << vec(i);

                if (i < T)
                    stream << ", ";
            }

            return stream;
        }
    };
}
#endif // SPACE_TIME_POINT_H
