
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
	};

	class StateStore : public IStore<State> {
	public:
	    StateStore();

	    void set(const State& value) override;
	    const State& get() const override;

	private:
	    State state_;
	};

	class GpsStore : public IStore<GpsBasicFixData> {
	public:
		GpsStore();

	    void set(const GpsBasicFixData& value) override;
	    const GpsBasicFixData& get() const override;

	private:
	    GpsBasicFixData data_;
	};


	struct GOATStore {
	    StateStore stateStore;
	    GpsStore   gpsStore;

	    void init() {
	        stateStore = StateStore{};
	        gpsStore   = GpsStore{};
	    }
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
