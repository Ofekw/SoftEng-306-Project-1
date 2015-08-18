$(document).ready(function(){

    var data = [ ]

    var table = $('#table').DataTable({
        ajax: '/ros/status.json'
    });


    table.ajax.url( '/ros/statusold.json' ).load();


});