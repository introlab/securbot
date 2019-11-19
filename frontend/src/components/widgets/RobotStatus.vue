<template>
  <div
    v-if="show"
    class="status-widget"
  >
    <div class="stat-col stat-group">
      <label>Desires</label>
      <ul>
        <li
          v-for="(desire, index) in desires"
          :key="index"
        >
          {{ desire.type }}
        </li>
      </ul>
    </div>
    <div class="stat-col resources">
      <p>CPU: {{ cpu }}%</p>
      <p>Mem: {{ mem }}%</p>
    </div>
    <div class="stat-col patrol">
      <p>Patrol: {{ patrol }}</p>
      <p>Waypoints: {{ reached }}/{{ planned }}</p>
    </div>
    <!--div class="stat-col wifi">
      <p>RSSI: {{ rssi }}dBm</p>
      <p>Noise: {{ noise }}dBm</p>
    </div-->
    <!--div class="stat-col stat-group">
      <label>Strategies</label>
      <ul>
        <li v-for="(strategie, index) in strategies" :key="index">
          {{ strategie }}
        </li>
      </ul>
    </div-->
  </div>
</template>

<script>
import { mapState } from 'vuex';

export default {
  name: 'robot-status',
  computed: mapState({
    cpu: state => state.client.robotStatus.resources.cpu.toFixed(1),
    mem: state => state.client.robotStatus.resources.mem.toFixed(1),
    rssi: state => state.client.robotStatus.wifi.rssi.toFixed(0),
    noise: state => state.client.robotStatus.wifi.noise.toFixed(0),
    strategies: state => state.client.robotStatus.intention.strategies,
    desires: state => state.client.robotStatus.intention.desires.filter(desire => desire.type !== 'Reverse'),
    patrol: state => state.client.patrolStatus.state,
    reached: state => state.client.patrolStatus.reached,
    planned: state => state.client.patrolStatus.planned,
    show: state => state.client.connectionState.robot === 'connected',
  }),
};
</script>

<style scoped>
* {
    padding: 0;
    margin: 0;
    box-sizing: border-box;
    color: whitesmoke;
}

.status-widget {
  margin-left: auto;
  display: flex;
  flex-direction: row;
  white-space: nowrap;
}

.stat-col {
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  border-left: whitesmoke solid 1px;
  margin-right: 20px;
  padding-left: 10px;
}

.stat-col:first-of-type {
  border: none;
  align-items: flex-end;
}

.stat-group > ul {
  display: flex;
  flex-direction: row;
}

.stat-group > ul > li {
  background-color: #0b5737;
  border-radius: 8px;
  position: relative;
  padding: 0px 8px;
  margin-right: 4px;
}

.stat-group > ul > li:last-of-type {
  margin-right: 0px;
}

.stat-pop {
  position: absolute;
  right: 0;
  top: 0;
  background-color: #083d27;
  display: none;
  flex-direction: column;
  border-radius: 8px;
}

.stat-group li:hover .stat-pop {
  display: flex;
}

.status-widget li {
  list-style: none;
}

.resources {
  min-width: 100px;
}

</style>
