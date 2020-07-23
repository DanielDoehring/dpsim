#pragma once

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	/// \brief MNA interface to be used by transfomer devices.
	class MNAOLTCInterface : public MNAInterface {
	public:
		typedef std::shared_ptr<MNAOLTCInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA section ####
		/// Open switch
		//virtual void mnaOpen() { }
		/// Close switch
		//virtual void mnaClose() { }
		/// Check if switch is closed
		virtual Bool mnaRatioChanged() = 0;
		/// Close switch if true and open switch if false
		//virtual void mnaSetClosed(Bool value) { }
		/// Stamps system matrix considering the defined switch position
		//virtual void mnaApplySwitchSystemMatrixStamp(Matrix& systemMatrix, Bool closed) { }
	};
}
