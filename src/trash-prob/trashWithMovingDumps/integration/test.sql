\pset pager off
\set ECHO queries

drop extension if exists vrptools;
create extension if not exists vrptools with schema public;

/*
select * from vrp_trashcollection(
  'select gid as id, round(st_x(st_transform(geom,4326))::numeric,6)::float8 as x, round(st_y(st_transform(geom,4326))::numeric,6)::float8 as y,
     0 as open, 360 as close, capacidad::integer as demand, 2 service, b.sid as street_id
     from contenedores_geom a, street_name_map b where a.gid=b.cid and left(a.cod_recorr,1)=''B'' order by gid',
  'select id, round(st_x(st_transform(geom,4326))::numeric,6)::float8 as x, round(st_y(st_transform(geom,4326))::numeric,6)::float8 as y,
     tw_open::integer as open, tw_close::integer as close
     from other_locs order by id',
  'select vid, start_id, dump_id, end_id, capacity, dump_service_time as dumpservicetime,
     tw_open::integer as starttime, tw_close::integer as endtime
     from vehicles',
  'select nfrom as from_id, nto as to_id, ttime from osrm_ttime_b order by id'
  );
*/

select * from  vrp_trashcollectioncheck( 'select * from containers limit 10 ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollectioncheck( 'select * from containers ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollectioncheck( 'select * from containers where demand <>0 limit 10 ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollectioncheck( 'select * from containers where demand <>0 ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollection( 'select * from containers where demand <> 0', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollection( 'select * from containers where demand <> 0', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',1);

select * from  vrp_trashcollection( 'select * from containers limit 10 ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollection( 'select * from containers ', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',0);

select * from  vrp_trashcollection( 'select * from containers where demand <> 0', 'select * from other_locs', 'select * from vehicles', 'select * from distance_matrix',2);


