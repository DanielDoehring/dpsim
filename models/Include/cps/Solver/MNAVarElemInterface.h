#pragma once

#include <cps/Config.h>
#include <cps/Definitions.h>
#include <cps/Solver/MNAInterface.h>

namespace CPS {
	/// \brief MNA interface to be used by transfomer devices.
	class MNAVarElemInterface : public MNAInterface {
	public:
		typedef std::shared_ptr<MNAVarElemInterface> Ptr;
		typedef std::vector<Ptr> List;

		// #### MNA section ####
		/// has the value of the element changed?
		virtual Bool ValueChanged() = 0;
	};
}