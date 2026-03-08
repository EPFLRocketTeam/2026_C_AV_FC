
#ifndef APP_DATA_H
#define APP_DATA_H

#include "Application/Data/fsm.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

namespace flight_computer {

	template <typename T>
	class IStore {
	public:
		virtual ~IStore() = default;

		virtual void set(const T& value) = 0;
		virtual const T& get() const = 0;
		virtual T* get_ref() = 0;
    
	};

	class StateStore : public IStore<State> {
	public:
	    StateStore();

	    void set(const State& value) override;
	    const State& get() const override;
	    State* get_ref() override;
    


	private:
	    State state_;
	};

	struct DataDump {
	    const State*  av_state;
	    const GpsBasicFixData* gps_state;
	};

	class GpsStore : public IStore<GpsBasicFixData> {
	public:
		GpsStore();

	    void set(const GpsBasicFixData& value) override;
	    const GpsBasicFixData& get() const override;
	    GpsBasicFixData* get_ref() override;

	    void setLon(int32_t lon);
		int32_t getLon() const;

		void setLat(int32_t lat);
		int32_t getLat() const;

		void setHAcc(uint32_t hAcc);
		uint32_t getHAcc() const;

		void setNumSV(uint8_t numSV);
		uint8_t getNumSV() const;

		void setFixType(GpsFixType type);
		GpsFixType getFixType() const;

		void setValidity(GpsValidityFlags valid);
		GpsValidityFlags getValidity() const;

		void setStatusFlags(GpsStatusFlags flags);
		GpsStatusFlags getStatusFlags() const;

	private:
	    GpsBasicFixData data_;
	};


	class GOATStore: public IStore<DataDump> {
	public:
	    GOATStore();

	    StateStore stateStore;
	    GpsStore   gpsStore;

	    void set(const DataDump& value) override;
	    const DataDump& get() const override;
	    DataDump* get_ref() override;

	private:
	    mutable DataDump data_;

	};




    class Data {
    public:
        State state;

        Data ();
    };
    
    class DataManager {
    public:
        static Data read ();

        static void setState (State state);
    };
};

#endif /* APP_DATA_H */
