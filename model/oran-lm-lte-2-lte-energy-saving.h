/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Logic Module for optimizing UE energy‐efficiency by adjusting transmit power.
 */

 #ifndef ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H
 #define ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H
 
 #include "oran-lm.h"
 #include "oran-data-repository.h"
 
 namespace ns3 {
 
 /**
  * \brief A Logic Module that monitors energy‐efficiency and issues
  *        TxPower adjustments to keep it near a target.
  */
 class OranLmLte2LteEnergySaving : public OranLm
 {
 public:
   static TypeId GetTypeId (void);
   OranLmLte2LteEnergySaving ();
   ~OranLmLte2LteEnergySaving () override;
 
   /**
    * \brief Run one execution of the control loop.
    * \return A vector of TxPower control commands to send.
    */
   std::vector<Ptr<OranCommand>> Run () override;
 
 private:
   double m_targetEfficiency; //!< Desired bits‐per‐joule
   double m_stepSize;         //!< Adjustment step for transmit power (dB)
 
 }; // class OranLmLte2LteEnergySaving
 
 } // namespace ns3
 
 #endif /* ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H */
 