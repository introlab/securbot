<template>
  <!-- Waypoint Table widget -->
  <div class="w-100 h-100">
    <!-- Overflow container -->
    <div class="mh-100 overflow-auto">
      <!-- Table -->
      <table
        id="waypoint-table"
        class="table table-borderless table-striped border-left border-right"
        style="text-align: center"
      >
        <!-- Header -->
        <thead class="text-white bg-green-sb">
          <th style="width:20%">
            #
          </th>
          <th style="width:20%">
            X
          </th>
          <th style="width:20%">
            Y
          </th>
          <th style="width:20%">
            Yaw
          </th>
          <th style="width:20%">
            Remove
          </th>
        </thead>
        <!-- Body -->
        <tbody>
          <!-- Table row per waypoint in list -->
          <template v-for="(waypoint, index) of waypointList">
            <tr
              :key="waypoint.dateTime"
              class="border-bottom"
            >
              <td style="width:20%">
                {{ (index+1).toFixed(0) }}
              </td>
              <td style="width:20%">
                {{ waypoint.x.toFixed(1) }}
              </td>
              <td style="width:20%">
                {{ waypoint.y.toFixed(1) }}
              </td>
              <td style="width:20%">
                {{ waypoint.yaw.toFixed(1) }}
              </td>
              <td style="width:20%">
                <!-- Remove button -->
                <button
                  :id="'removeBtn'+index"
                  type="button"
                  class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                  @click="removeWaypointFromList(index)"
                >
                  <img
                    src="~/open-iconic/svg/trash.svg"
                    alt=""
                    style="width:12px;height:12px"
                  >
                </button>
              </td>
            </tr>
          </template>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script>
/**
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import { mapState } from 'vuex';

export default {
  name: 'waypoint-table',
  computed: mapState({
    waypointList: state => state.patrol.waypointList,
  }),
  methods: {
    removeWaypointFromList(index) {
      this.$store.commit('removeWaypoint', index);
    },
  },
};
</script>

<style>
.bg-trash{
  display: block;
  background: url("~/open-iconic/svg/trash.svg");
  fill: black;
}
</style>
