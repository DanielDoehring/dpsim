#include <cstdio>
#include <cstdlib>

#include <villas/sample.h>
#include <villas/shmem.h>

#include "ShmemInterface.h"

using namespace DPsim;

void ShmemInterface::init(const char* wname, const char* rname, struct shmem_conf* conf) {
	/* using a static shmem_conf as a default argument for the constructor
	 * doesn't seem to work, so use this as a workaround */
	if (shmem_int_open(wname, rname, &this->mShmem, conf) < 0) {
		std::perror("Failed to open/map shared memory object");
		std::exit(1);
	}
	mSeq = 0;
	if (shmem_int_alloc(&mShmem, &mLastSample, 1) < 0) {
		std::cerr << "Failed to allocate single sample from shmem pool" << std::endl;
		std::exit(1);
	}
	mLastSample->sequence = 0;
	mLastSample->ts.origin.tv_sec = 0;
	mLastSample->ts.origin.tv_nsec = 0;
	std::memset(&mLastSample->data, 0, mLastSample->capacity * sizeof(float));
}

ShmemInterface::ShmemInterface(const char* wname, const char* rname) {
	struct shmem_conf conf;
	conf.queuelen = 512;
	conf.samplelen = 64;
	conf.polling = 0;
	init(wname, rname, &conf);
}

ShmemInterface::ShmemInterface(const char* wname, const char *rname, struct shmem_conf* conf) {
	init(wname, rname, conf);
}

ShmemInterface::~ShmemInterface() {
	shmem_int_close(&mShmem);
}

void ShmemInterface::readValues() {
	if (!mInit) {
		mInit = 1;
		return;
	}
	struct sample *sample = nullptr;
	int ret = 0;
	try {
		while (ret == 0)
			ret = shmem_int_read(&mShmem, &sample, 1);
		if (ret < 0) {
			std::cerr << "Fatal error: failed to read sample from shmem interface" << std::endl;
			std::exit(1);
		}
		for (ExtComponent extComp : mExtComponents) {
			if (extComp.realIdx >= sample->length || extComp.imagIdx >= sample->length) {
				std::cerr << "Fatal error: incomplete data received from shmem interface" << std::endl;
				std::exit(1);
			}
			// TODO integer format?
			Real real = sample->data[extComp.realIdx].f;
			Real imag = sample->data[extComp.imagIdx].f;

			ExternalCurrentSource *ecs = dynamic_cast<ExternalCurrentSource*>(extComp.comp);
			if (ecs)
				ecs->setCurrent(real, imag);
			ExternalVoltageSource *evs = dynamic_cast<ExternalVoltageSource*>(extComp.comp);
			if (evs)
				evs->setVoltage(real, imag);
		}
		sample_put(sample);
	} catch (std::exception& exc) {
		/* probably won't happen (if the timer expires while we're still reading data,
		 * we have a bigger problem somewhere else), but nevertheless, make sure that
		 * we're not leaking memory from the queue pool */
		if (sample)
			sample_put(sample);
		throw exc;
	}
}

void ShmemInterface::writeValues(SystemModel& model) {
	struct sample *sample = nullptr;
	Int len = 0, ret = 0;
	bool done = false;
	try {
		if (shmem_int_alloc(&mShmem, &sample, 1) < 1) {
			std::cerr << "fatal error: shmem pool underrun" << std::endl;
			std::exit(1);
		}
		for (auto vd : mExportedVoltages) {
			Real real = 0, imag = 0;
			if (vd.from > 0) {
				real += model.getRealFromLeftSideVector(vd.from-1);
				imag += model.getImagFromLeftSideVector(vd.from-1);
			}
			if (vd.to > 0) {
				real -= model.getRealFromLeftSideVector(vd.to-1);
				imag -= model.getImagFromLeftSideVector(vd.to-1);
			}
			if (vd.realIdx >= sample->capacity || vd.imagIdx >= sample->capacity) {
				std::cerr << "fatal error: not enough space in allocated struct sample" << std::endl;
				std::exit(1);
			}
			sample->data[vd.realIdx].f = real;
			sample->data[vd.imagIdx].f = imag;
			if (vd.realIdx > len)
				len = vd.realIdx;
			if (vd.imagIdx > len)
				len = vd.imagIdx;
		}
		for (auto cd : mExportedCurrents) {
			Complex current = cd.comp->getCurrent(model);
			if (cd.realIdx >= sample->capacity || cd.imagIdx >= sample->capacity) {
				std::cerr << "fatal error: not enough space in allocated struct sample" << std::endl;
				std::exit(1);
			}
			sample->data[cd.realIdx].f = current.real();
			sample->data[cd.imagIdx].f = current.imag();
			if (cd.realIdx > len)
				len = cd.realIdx;
			if (cd.imagIdx > len)
				len = cd.imagIdx;
		}
		sample->length = len+1;
		sample->sequence = mSeq++;
		clock_gettime(CLOCK_MONOTONIC, &sample->ts.origin);
		done = true;
		while (ret == 0)
			ret = shmem_int_write(&mShmem, &sample, 1);
		if (ret < 0)
			std::cerr << "Failed to write samples to shmem interface" << std::endl;
		sample_copy(mLastSample, sample);
	} catch (std::exception& exc) {
		/* We need to at least send something, so determine where exactly the
		 * timer expired and either resend the last successfully sent sample or
		 * just try to send this one again.
		 * TODO: can this be handled better? */
		if (!done)
			sample = mLastSample;
		while (ret == 0)
			ret = shmem_int_write(&mShmem, &sample, 1);
		if (ret < 0)
			std::cerr << "Failed to write samples to shmem interface" << std::endl;
		/* Don't throw here, because we managed to send something */
	}
}