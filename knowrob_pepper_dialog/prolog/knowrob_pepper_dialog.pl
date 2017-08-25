/*
  Copyright (C) 2017 Sascha Jongebloed
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Sascha Jongebloed
@license BSD

*/

:- module(knowrob_pepper_dialog,
    [
      pepper_answer/4
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

%% pepper_answer(+Question, -Answer, -Animation, -Picture) is semidet.
%
% Returns a answer to a question in nl
%
% @param Question       Question to pepper in nl
% @param Answer         Answer in nl
% @param Animation      Path to animation for pepper
% @param Picture        Path to a picture to show on peppers tablet
%
pepper_answer('Dance', 'Yes i can', 'animations/Stand/Gestures/Hey_1', Picture).
pepper_answer('dance', 'Yes i can', 'animations/Stand/Gestures/Hey_1', Picture).
pepper_answer('Dancing', 'Yes i can', 'animations/Stand/Gestures/Hey_1', Picture).
pepper_answer('dancing', 'Yes i can', 'animations/Stand/Gestures/Hey_1', Picture).
pepper_answer('show me a map', 'sure. look at my tablet', Animation, 'http://www.uni-bremen.de/fileadmin/_processed_/csm_20160810-uni_lageplan_3_Not_5ed6aabebd.png').
pepper_answer('Show me a map', 'sure. look at my tablet', Animation, 'http://www.uni-bremen.de/fileadmin/_processed_/csm_20160810-uni_lageplan_3_Not_5ed6aabebd.png').
pepper_answer('greet the audience', 'Hi everyone, my name is Pepper. Welcome in the institute of artificial intelligence Bremen', Animation, Picture).
pepper_answer('who are you', 'My name is Pepper and i was build by the company Alderbaraan. ', Animation, Picture).


    