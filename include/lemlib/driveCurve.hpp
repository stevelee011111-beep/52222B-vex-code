namespace lemlib {

/**
 * @brief abstract DriveCurve class
 *
 * This is an abstract class to enable users to provide their own, custom drive
 * curves for LemLib to use.
 */
class DriveCurve {
    public:
        /**
         * @brief Processes an input and returns an output
         *
         * This is a pure virtual function that needs to be overriden by child classes
         *
         * @param input the input to process
         * @return float output
         */
        virtual float curve(float input, float maxVelo = 127) = 0;
};

/**
 * @brief  ExpoDriveCurve class. Inherits from the DriveCurve class. This is a exponential drive curve.
 *
 * see https://www.desmos.com/calculator/umicbymbnl for an interactive graph
 * see https://www.vexforum.com/t/expo-drive-lemlibs-implementation for a detailed explanation
 */
class ExpoDriveCurve : public DriveCurve {
    public:
        /**
         * @brief Construct a new Expo Drive Curve object
         *
         * see https://www.desmos.com/calculator/umicbymbnl for an interactive graph
         * see https://www.vexforum.com/t/expo-drive-lemlibs-implementation for a detailed explanation
         *
         * @param deadband range where input is considered to be input
         * @param minOutput the minimum output that can be returned
         * @param curve how "curved" the graph is
         */
        ExpoDriveCurve(float deadband, float minOutput, float curve);

        /**
         * @brief curve an input
         *
         * @param input the input to curve
         * @return float the curved output
         */
        float curve(float input, float maxVelo = 127);
        
    private:
        const float deadband = 0;
        const float minOutput = 0;
        const float curveGain = 1;
};
} // namespace lemlib