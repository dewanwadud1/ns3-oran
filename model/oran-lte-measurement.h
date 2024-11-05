#ifndef ORAN_LTE_MEASUREMENT_H
#define ORAN_LTE_MEASUREMENT_H

#include "ns3/object.h"
#include "ns3/lte-module.h"

namespace ns3 {

struct LteMeasurements
{
    double rsrp;
    double rsrq;
    double sinr;
    double antennaTilt;
    double antennaGain;
    double txPower;
};

class OranLteMeasurement : public Object
{
public:
    static TypeId GetTypeId(void);
    OranLteMeasurement();
    virtual ~OranLteMeasurement();

    void Setup(Ptr<LteUeNetDevice> ueDevice, Ptr<LteEnbNetDevice> enbDevice);
    void GetMeasurements();

private:
    Ptr<LteUeNetDevice> m_ueDevice;
    Ptr<LteEnbNetDevice> m_enbDevice;
    LteMeasurements m_measurements;

    void CalculateMeasurements();
};

} // namespace ns3

#endif /* ORAN_LTE_MEASUREMENT_H */

