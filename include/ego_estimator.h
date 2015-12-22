#ifndef EGO_ESTIMATOR_H
#define EGO_ESTIMATOR_H

#include <lms/datamanager.h>
#include <lms/module.h>

/**
 * @brief LMS module ego_estimator
 **/
class EgoEstimator : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
};

#endif // EGO_ESTIMATOR_H
