<template>
  <div
    id="operator-layout"
    class="vh-100"
  >
    <navbar />
    <div style="height:calc(100% - 64px)">
      <router-view />
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';
import Navbar from './widgets/Navbar';

/**
 * This component is the main root component (after the app). It includes the navbar and the view
 * routing is done here.
 *
 * @since 0.1.0
 * @displayName Layout
 */
export default {
  name: 'layout',
  components: {
    Navbar,
  },
  computed: mapState({
    myId: state => state.client.myId,
    robotId: state => state.client.robotId,
    robotList: state => state.client.robotList,
    cameraId: state => state.htmlElement.cameraId,
    mapId: state => state.htmlElement.mapId,
  }),
  mounted() {
    this.$store.dispatch('client/connectToServer');
  },
};
</script>

<style>
/* Changes to the bootstrap CSS */
.jumbotron {
  margin-bottom: 0;
}
.container-fluid {
  height: 100%
}
/*Custom CSS element (SecurBot)*/
.shadow-sb {
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 54, 5, 0.19);
}
.bg-black-sb {
  background-color: black;
}
.bg-green-sb {
  background-color:#00A759
}
.b-collapse-sb {
  border-collapse: collapse;
}
::-webkit-scrollbar {
  width: 10px;
}
::-webkit-scrollbar-thumb {
  background-color: darkgrey;
  outline: 1px solid slategrey;
}
::-webkit-scrollbar-track {
    -webkit-box-shadow: inset 0 0 3px rgba(0,0,0,0.3);
}
</style>
