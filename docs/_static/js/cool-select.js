$( document ).ready(function() {

  $( "table" ).addClass( "table table-striped" );

  $( "#language-select" ).click(function() {
    $( "#language-list" ).slideToggle();
    $("#arrow").toggleClass("rotate-arrow");
  });
});
