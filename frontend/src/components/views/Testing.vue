<template>
  <b-jumbotron
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100 bg-transparent"
  >
    <b-row class="h-100">
      <b-col
        md="4"
        class="mh-100 overflow-auto"
      >
        <b-table
          sticky-header
          striped
          hover
          fixed
          head-variant="dark"
          :items="elements"
        />
      </b-col>
    </b-row>
  </b-jumbotron>
</template>

<script>
export default {
  name: 'testing',
  data() {
    return {
      headers: ['index', 'random'],
    };
  },
  computed: {
    elements() {
      const tt = [];
      for (let i = 0; i < 35; i++) {
        const t = {
          t1: Math.random(),
          t2: Math.random(),
        };
        tt[i] = t;
      }
      return tt;
    },
  },
};
</script>

<style scoped>
/* .fixed-header tbody{
  display:block;
  overflow:auto;
  height:200px;
  width:100%;
}
.fixed-header thead tr{
  display:block;
  width: 100%;
} */
</style>
