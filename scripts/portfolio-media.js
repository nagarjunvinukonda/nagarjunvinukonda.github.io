/* Silent GIF-like playback plus the original portfolio scroll-in motion. */
(function () {
  var preference = window.matchMedia('(prefers-reduced-motion: reduce)');

  if (!preference.matches) {
    document.querySelectorAll('.portfolio-card').forEach(function (card) {
      var panel = card.querySelector('.demo-panel');
      var text = card.querySelector('.col-md-9');
      if (panel) {
        panel.setAttribute('data-aos', 'fade-right');
        panel.setAttribute('data-aos-offset', '45');
        panel.setAttribute('data-aos-duration', '550');
      }
      if (text) {
        text.setAttribute('data-aos', 'fade-left');
        text.setAttribute('data-aos-offset', '45');
        text.setAttribute('data-aos-duration', '550');
      }
    });
    document.querySelectorAll('.portfolio-card .btn').forEach(function (button) {
      button.setAttribute('data-aos', 'zoom-in');
      button.setAttribute('data-aos-anchor', 'data-aos-anchor');
    });
    var contactCard = document.querySelector('#contact .card');
    if (contactCard) contactCard.setAttribute('data-aos', 'zoom-in');
  }

  function updateMotion() {
    document.querySelectorAll('.demo video').forEach(function (video) {
      video.autoplay = !preference.matches;
      if (preference.matches) video.pause();
      else video.play().catch(function () { /* Preserve poster when autoplay is unavailable. */ });
    });
  }
  updateMotion();
  if (preference.addEventListener) preference.addEventListener('change', updateMotion);
})();
