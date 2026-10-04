/* GIF-like demo loops, with user control and reduced-motion support. */
document.querySelectorAll('.demo').forEach(function (demo) {
  var video = demo.querySelector('video');
  var button = demo.querySelector('button');
  var reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
  function label() { button.textContent = video.paused ? 'Play animation' : 'Pause animation'; button.setAttribute('aria-label', button.textContent + ': ' + video.getAttribute('aria-label')); }
  if (reduceMotion.matches) { video.autoplay = false; video.pause(); }
  button.addEventListener('click', function () {
    if (video.paused) { video.play().catch(function () { video.controls = true; }); }
    else { video.pause(); }
  });
  video.addEventListener('play', label);
  video.addEventListener('pause', label);
  video.addEventListener('error', function () { button.textContent = 'View source below'; video.controls = true; });
  label();
});
