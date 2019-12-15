<template>
  <b-container fluid>
    <b-row class="h-100">
      <div
        class="h-100 mega-col"
      >
        <teleop-view />
      </div>
      <div
        class="mega-col full-screen-row"
      >
        <patrol-view />
      </div>
    </b-row>
  </b-container>
</template>

<script>
import TeleopView from './Teleop';
import PatrolView from './PatrolPlanner';

export default {
  name: 'mega-genial',
  components: {
    TeleopView,
    PatrolView,
  },
};
</script>

<style scoped>
.mega-col {
  position: relative;
  width: 100%;
  padding-left: 15px;
  padding-right: 15px;
}
.full-screen-row {
  height: calc( 100% + 64px );
}
@media (min-width: 2800px) {
  .mega-col {
    -ms-flex: 0 0 50%;
    -webkit-box-flex: 0 0 50%;
    flex: 0 0 50%;
    max-width: 50%;
  }
  .full-screen-row {
    height: 100%;
  }
}
</style>
